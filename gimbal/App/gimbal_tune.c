/* Online-tunable parameters (Keil Watch; Bluetooth "S <name> <value>", see Gimbal_SetTune) and
 * the notes on why each default is what it is. gimbal_task.c reads them every control cycle;
 * the table at the end maps the Bluetooth names (docs/GIMBAL_TUNING.md section 4). */

#include "gimbal_tune.h"
#include "gimbal_task.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

/* ===== 跟踪滤波 + 延迟补偿 + 速度前馈（参数可在 Keil Watch 里直接改） =====
 * 每帧换算出靶的方向（yaw：拍照时 IMU yaw + 误差 = 世界方位；pitch：拍照时电机角 + 误差
 * + IMU 俯仰 = 对地俯仰），用 alpha-beta 滤波估计方向和变化速度（track_filter.h）。
 * 云台每 2 ms 跟随预测值 x + v*(now - 拍照时刻)，并把 v 作为速度前馈加到电机指令上：
 * 帧间不再一跳一跳，延迟被补上，车横向走过靶时不再靠积累误差才转起来。
 * tune_vision_latency_ms：拍照到云台收到这一帧的时间，上限 200。上靶过冲->加大，欠补->减小。
 * tune_track_alpha：每帧信测量多少（大=跟手、噪声大），0.3~0.7。
 * tune_track_beta：速度学得多快，常用 alpha^2/(2-alpha)。
 * tune_track_ff：速度前馈比例，0 关闭（用来对比），1 全开。
 * tune_yaw_kp_scale：yaw 位置环 Kp 的倍数。控制板在 yaw 转台上，陀螺仪前馈（增益 K=7）同时在
 *   抑制云台自己的转动，任何转速指令实际只剩 1/(1+K)：Kp 2.8 rpm/deg 只相当于 2.1 deg/s 每度，
 *   yaw 很"软"（gimbal/tools/track_sim.py）。实车 2026-10-24：2 稳；4 静止时跟着单帧视觉噪声
 *   小幅来回抽（rms 0.01 -> 0.09 度），所以默认 2。 */
volatile uint32_t tune_vision_latency_ms = 80U;
volatile float tune_track_alpha = 0.5f;
volatile float tune_track_beta = 0.17f;
volatile float tune_track_ff = 1.0f;
volatile float tune_yaw_kp_scale = 2.0f;
/* ===== yaw 两层环（tune_yaw_two_loop = 1，默认；0 = 上面的旧结构） =====
 * 外环：期望世界转速 w* = tune_track_ff * 跟踪速度 + tune_yaw_kpos * 方位误差（deg/s）。
 * 内环：电机 rpm = (w* - tune_yaw_chassis_ff * 底盘 gz + tune_yaw_krate * (w* - 转台陀螺仪)) / 6。
 * 底盘 gz 来自 $C 帧（50 Hz），直接抵消转弯时的车身转动；krate 7 与旧陀螺仪前馈同等强度。
 * 仿真（track_sim.py）：90 度弯激光甩动 2.7 度 -> 0.34 度（kpos 8）。
 * 上板前先确认 gz 方向：手转车身时 dbg_chassis_gz_deg_s 与 dbg_yaw_motor_rate_deg_s 应反号
 * （云台反向转来保持朝向）；同号就把 tune_yaw_chassis_ff 改成 -1。
 * 实车整圈 2026-10-24（加 settle/posmax 后）：转弯中落后 0.7~1.8 度（旧结构 2.2~2.9），
 * 远边 rms 1.40 度（旧结构 1.92），没有振荡，所以默认用两层环。 */
volatile uint8_t tune_yaw_two_loop = 1U;
volatile float tune_yaw_kpos = 8.0f;          /* deg/s per deg */
volatile float tune_yaw_krate = 7.0f;
volatile float tune_yaw_chassis_ff = 1.0f;    /* x chassis gz; -1 if its sign is opposite, 0 off */
/* 大误差重新锁靶（出弯后约 5 度）时两层环曾振荡（实车 2026-10-24：电机打满、120 deg/s 来回甩）：
 * kpos x 误差没有上限，而云台快速甩动时的几帧又让跟踪器学到 +-40 deg/s 的假速度。
 * tune_yaw_pos_rate_max：两层环位置项 kpos x 误差的上限（deg/s），跟踪速度和底盘前馈不受限。
 * tune_track_settle_ms：跟踪器每次（重新）开始后这段时间只跟位置、不学速度（车速前馈照常）。 */
volatile float tune_yaw_pos_rate_max = 40.0f;
/* tune_yaw_rate_ki（"ratei"，1/s，0 = 关）：两层环内环的积分，加在 krate 项旁边。yaw 电机带着转台像约
 * 150 ms 的一阶滞后（遥测通道 9，出厂速度环），弯里转台被车身带着落后约 1 度；积分累加 (w* - 转台陀螺仪)
 * 补电机欠下的转速。仿真（rate_integral_sim.py，纯一阶滞后）只给 -4 %（20）～-27 %（50），实车好得多：
 * 3 圈 2026-10-28（各 12 个弯，gimbal/tools/telemetry_log.py report i0 i20 i40 i60）：
 *   ratei      弯里 yaw rms  弯里 p99 / 最大   弯峰平均   直道 yaw rms / p99   直道转台转速偏差 rms
 *     0          0.52        1.47 / 1.80      1.16       0.32 / 1.47        4.3 deg/s
 *    20          0.44        1.47 / 1.66      1.06       0.27 / 1.09        4.4
 *    40          0.29        1.16 / 1.35      0.85       0.27 / 1.06        4.1
 *    60          0.31        0.96 / 1.23      0.73       0.29 / 1.36        5.1（直道开始多晃）
 * 取 40：弯里大部分改善已拿到、直道最好，留稳定余量（两层环以前有过偶发振荡）。
 * |方位误差| >= YAW_RATE_I_ZONE_DEG（重新锁靶）时保持不积；输出打满时不往打满的方向积；
 * 限幅 tune_yaw_rate_i_max（"rateimax"，deg/s）。 */
volatile float tune_yaw_rate_ki = 40.0f;
volatile float tune_yaw_rate_i_max = 60.0f;
/* tune_yaw_gz_lead_s（"gzlead"，s，0 = 关）：转弯时底盘 gz 前馈超前这么多秒（gz + lead x dgz/dt），
 * 只在 $C 报底盘转弯时用，直道上车头的小晃动不被放大。实车 3 圈 2026-10-28：最大的 yaw 误差
 * （第 4 弯 1.3 度）出在弯一开始，gz 0.15 s 内 25 -> 104 deg/s，电机滞后约 130 ms 才反转跟上。
 * 3 圈 2026-10-29：0 -> 0.1 s，弯开始 0.6 s 内 yaw 峰值平均 0.51 -> 0.40、最大 0.78 -> 0.71 度，
 * 第 4 弯 0.58~0.78 -> 0.52~0.54 度，弯内 yaw rms 0.23 -> 0.20（最初记的 0.84 -> 0.38 是工具把
 * tsel 1 时的通道 10 当成底盘 gz 对齐、弯的位置错了约 1 s 算出来的）。 */
volatile float tune_yaw_gz_lead_s = 0.1f;
/* tune_yaw_gz_tail_ms（"gztail"，ms，0 = 关）：转弯标志熄灭后 gz 超前再保持这么久。弯尾 gz 约 0.1 s 内
 * 22 -> 8 deg/s，标志正好在这时熄灭，超前没加上；电机滞后，弯尾后 0~0.1 s 转台世界角速度比指令多转
 * 1.4~2.0 deg/s（2026-10-29 所有跑次、所有弯；第 4 弯出口平均 -0.47 度误差的约一半）。
 * 3 圈 0 / 150 / 300（0 两组、150 两组）：直道 yaw rms 0.16 / 0.13~0.14 / 0.14，第 4 弯出口 yaw rms
 * 0.28~0.30 / 0.24~0.27 / 0.35，全程最大 0.75~1.07 / 0.83~0.84 / 1.21（300 时出弯循迹的小修正也被
 * 超前放大）-> 默认 150。 */
volatile uint32_t tune_yaw_gz_tail_ms = 150U;
/* tune_telem_select（"tsel"）：遥测通道 9/10 的内容。0 = yaw 电机实测 rpm / 底盘 gz（默认）；
 * 1 = 云台 IMU pitch (deg) / pitch 跟踪速度 (deg/s)；2 = pitch 车速前馈 (deg/s) / 车速 (cm/s)；
 * 3 = gz 超前量 (deg/s) / dgz/dt (deg/s^2)；4 = yaw 车速前馈 (deg/s) / 前馈用的车速 (cm/s，含 vlead)。
 * 诊断用，带宽不变。 */
volatile uint8_t tune_telem_select = 0U;
volatile uint32_t tune_track_settle_ms = 300U;
/* tune_track_rate_clip_deg（"rateclip"，0 = 关）：跟踪器学速度时单帧残差的上限。整圈 2026-10-24 有一次
 * yaw 在侧边振荡（3.8 度、电机打满）：之前 0.1 s 内跟踪速度被几帧跳动的视觉推着从 +4 跳到 +20 deg/s。
 * gimbal/tools/jitter_sim.py（K230 每帧 20~86 ms、3 % 跳点）：最坏误差 2.6 -> 1.3~1.5 度，无跳点时不变。 */
volatile float tune_track_rate_clip_deg = 0.4f;
/* tune_turn_hold：0 = 转弯时视觉照常（默认）；1 = 底盘转弯（$C mode=1）时暂停视觉、按 IMU 保持方向
 * （原来的做法）。暂停约 1.5 s/弯，车仍以 0.14 m/s 前进约 0.2 m，出弯第一帧 yaw 约 -5.5 度、
 * pitch +4 度；不暂停时出弯误差 <= 1.6 度、弯内视觉有效 71~100%（实车整圈 2026-10-24）。
 * 转弯时跟踪器只修位置、不学速度（车身转动由底盘 gz 前馈抵消），出弯 tune_track_settle_ms 后恢复：
 * 一个弯里曾学到 +-30 deg/s 的假速度而振荡。
 * 底盘链路断开（收到过后超过 GIMBAL_CHASSIS_TIMEOUT_MS 没帧）只响蜂鸣器，视觉照常跟踪，
 * 底盘相关的前馈自然变 0：起步时一次 0.3 s 的断开曾让视觉停 0.5 s、误差 15 度。 */
volatile uint8_t tune_turn_hold = 0U;
/* ===== 车速前馈（tune_veh_ff，默认 +1 = 打开；0 = 关闭） =====
 * 车往前走时靶的方位变化率 = v * sin(phi) / r（deg/s 换算后），v = 底盘编码器车速（$C v2），
 * r = K230 给的靶距离（$V v2），phi = 激光方向相对车头的角度 = yaw 电机角 - tune_yaw_fwd_deg。
 * 作为跟踪滤波器的已知输入：起步、停车时不用等视觉"学"出速度（仿真：起步落后 2.9 -> 0.4 度）。
 * tune_yaw_fwd_deg：激光对准车头正前方时的 yaw 电机角。yaw 电机编码器是绝对式的（重启前后 F 读数
 *   31.74 / 31.69 / 31.89 度），所以这是个机械常数 VEH_CHASSIS_FORWARD_DEG；云台或电机重装后，把靶放在
 *   车正前方、锁定后蓝牙发 "F" 重新标定。以前用「上电角 - 90 度」，实际指向车尾且随上电摆车偏 ±13 度
 *   （当时靠 vehff = -1、pitchh = +0.182 抵消符号）。
 * 实车 2026-10-24（DRIVE 20 60，靶在侧面约 0.5 m）：前馈开时最大误差 3.4 -> 约 1 度、rms 1.2 -> 0.35 度；
 *   符号反了误差翻倍。用真实车头方向后符号只由硬件决定，与靶在车的哪一侧无关。
 * tune_vision_use_age = 1：延迟改用 K230 报的本帧耗时 + tune_vision_extra_ms（拍照前的曝光读出）。 */
volatile float tune_veh_ff = 1.0f;
/* tune_veh_pitch_h（m，"pitchh"，0 = 关）：靶心比相机高多少，负号来自 pitch 方向。练习场地量得 0.182 m，
 * 实车 3 圈（2026-10-24）：pitch rms 0.44 -> 0.35 度，朝靶那条侧边出弯尖峰 +1.0~1.3 -> +0.8~0.9，
 * 背靶那条 -1.0~-1.25 -> -0.3~-0.5 度。换场地重新量 h（符号不变：靶心高于相机就是负值）。
 * 车朝靶开时仰角变大：仰角变化率 = h * v * cos(phi) / r^2，作为 pitch 跟踪器的已知输入和前馈，
 * 和 yaw 的车速前馈一样。整圈 2026-10-24：yaw 有车速前馈后 pitch 误差（rms 0.44 度）已与 yaw 相当，
 * 侧边（车朝靶 / 背靶开，cos(phi) 大）最明显。 */
volatile float tune_veh_pitch_h = -0.182f;
/* tune_veh_speed_lead_s（"vlead"，s，0 = 关）：车速前馈用 v + vlead x dv/dt。$C 里的车速是编码器速度
 * 经 30 ms 低通，再加 20 ms 一帧和链路延迟，比真车速晚 70~90 ms；匀速时无所谓，加速时（起步、出弯
 * 0.14 -> 0.35 m/s，0.8 m/s^2）离靶近的地方方位转速就少估。实车 3 圈 2026-10-29：最大的 yaw 误差
 * （起点边、出第 4 弯后 0.8~1.0 度，落后）都出在加速段，与转台转速本身不成比例（不是固定延迟）。
 * vlead 0 / 0.08 / 0.05：加速段 yaw rms 0.26~0.50 / 0.09 / 0.13 度，起点边最大 0.61~1.03 / 0.45 / 0.29，
 * 全程最大 0.82~1.04 / 0.90 / 0.75；0.08 出第 4 弯（离靶最近）rms 0.34（0.05 为 0.28）-> 默认 0.05。
 * 出第 4 弯仍有 ~0.5 度：不是车速延迟（底盘改发指令车速反而更差），里程计重建的几何显示前馈
 * 只差 2~5%；尖峰跟着弯尾底盘转动急停走，与起步加速度无关（见 tune_yaw_gz_tail_ms）。 */
volatile float tune_veh_speed_lead_s = 0.05f;
volatile float tune_yaw_fwd_deg = VEH_CHASSIS_FORWARD_DEG;
volatile uint8_t tune_vision_use_age = 0U;
volatile uint32_t tune_vision_extra_ms = 40U;
/* "pkp" in the tune table. 3 laps 2026-10-29 (tsel 1): the vision pitch error follows the pitch
 * loop error (motor behind its target angle) with slope 0.9-1.0, corr 0.62-0.65, while the
 * stage tilt the IMU sees barely correlates (0.02-0.11): the pitch motor lags its target.
 *   pkp      straight pitch rms   corner pitch rms   edge after corner 1 max   static jitter
 *   2.8      0.34 deg             0.24               1.51                      0.004
 *   4        0.29                 0.22               1.00                      0.003
 *   5.5      0.26                 0.19               0.81                      0.009
 *   7        0.23                 0.17               0.59                      0.004 (0.022 on a low battery)
 * No oscillation up to 7 (the <0.2 s part of the pitch error stays 0.064-0.067 deg); not raised
 * further because the low-battery jitter shows the margin shrinking. (Rows recomputed with the
 * corrected clock alignment: straight pitch rms 0.36 / 0.30 / 0.27 / 0.22.) */
volatile float g_pitch_kp = 7.0f;

/* ===================== 在线调参（蓝牙） ===================== */
typedef struct {
    const char *name;
    volatile float *f;                 /* exactly one of f / u32 / u8 is set */
    volatile uint32_t *u32;
    volatile uint8_t *u8;
    float lo, hi;
} tune_entry_t;

static const tune_entry_t g_tunes[] = {
    {"latency",   NULL, &tune_vision_latency_ms, NULL, 0.0f, (float)VISION_LATENCY_MAX_MS},
    {"alpha",     &tune_track_alpha, NULL, NULL, 0.05f, 1.0f},
    {"beta",      &tune_track_beta, NULL, NULL, 0.0f, 1.0f},
    {"ff",        &tune_track_ff, NULL, NULL, 0.0f, 2.0f},
    {"kpscale",   &tune_yaw_kp_scale, NULL, NULL, 0.1f, 10.0f},
    {"twoloop",   NULL, NULL, &tune_yaw_two_loop, 0.0f, 1.0f},
    {"kpos",      &tune_yaw_kpos, NULL, NULL, 0.0f, 40.0f},
    {"krate",     &tune_yaw_krate, NULL, NULL, 0.0f, 20.0f},
    {"chassisff", &tune_yaw_chassis_ff, NULL, NULL, -2.0f, 2.0f},
    {"vehff",     &tune_veh_ff, NULL, NULL, -1.5f, 1.5f},
    {"yawfwd",    &tune_yaw_fwd_deg, NULL, NULL, -100000.0f, 100000.0f},
    {"useage",    NULL, NULL, &tune_vision_use_age, 0.0f, 1.0f},
    {"latencyx",  NULL, &tune_vision_extra_ms, NULL, 0.0f, 150.0f},
    {"settle",    NULL, &tune_track_settle_ms, NULL, 0.0f, 1000.0f},
    {"posmax",    &tune_yaw_pos_rate_max, NULL, NULL, 5.0f, 200.0f},
    {"turnhold",  NULL, NULL, &tune_turn_hold, 0.0f, 1.0f},
    {"rateclip",  &tune_track_rate_clip_deg, NULL, NULL, 0.0f, 5.0f},
    {"pitchh",    &tune_veh_pitch_h, NULL, NULL, -2.0f, 2.0f},
    {"ratei",     &tune_yaw_rate_ki, NULL, NULL, 0.0f, 100.0f},
    {"rateimax",  &tune_yaw_rate_i_max, NULL, NULL, 0.0f, 200.0f},
    {"gzlead",    &tune_yaw_gz_lead_s, NULL, NULL, 0.0f, 0.3f},
    {"gztail",    NULL, &tune_yaw_gz_tail_ms, NULL, 0.0f, 500.0f},
    {"tsel",      NULL, NULL, &tune_telem_select, 0.0f, 4.0f},
    {"vlead",     &tune_veh_speed_lead_s, NULL, NULL, 0.0f, 0.3f},
    {"pkp",       &g_pitch_kp, NULL, NULL, 0.5f, 10.0f},
};

static bool same_name(const char *a, const char *b)
{
    for (; *a && *b; ++a, ++b) {
        char x = *a, y = *b;
        if (x >= 'A' && x <= 'Z') x = (char)(x - 'A' + 'a');
        if (y >= 'A' && y <= 'Z') y = (char)(y - 'A' + 'a');
        if (x != y) return false;
    }
    return *a == *b;
}

int Gimbal_SetTune(const char *name, float value)
{
    for (unsigned i = 0; i < sizeof(g_tunes) / sizeof(g_tunes[0]); ++i) {
        const tune_entry_t *t = &g_tunes[i];
        if (!same_name(name, t->name)) continue;
        if (!isfinite(value) || value < t->lo || value > t->hi) return -2;
        if (t->f) *t->f = value;
        else if (t->u32) *t->u32 = (uint32_t)lroundf(value);
        else *t->u8 = (uint8_t)lroundf(value);   /* on/off switches and tsel 0..4 */
        return 0;
    }
    return -1;
}

int Gimbal_FormatTunes(char *out, unsigned capacity)
{
    int n = 0;
    if (!out || !capacity) return 0;
    out[0] = '\0';
    for (unsigned i = 0; i < sizeof(g_tunes) / sizeof(g_tunes[0]) && n < (int)capacity; ++i) {
        const tune_entry_t *t = &g_tunes[i];
        const float v = t->f ? *t->f : t->u32 ? (float)*t->u32 : (float)*t->u8;
        const int w = snprintf(out + n, capacity - (unsigned)n, "%s%s=%.3g", i ? " " : "", t->name, (double)v);
        if (w < 0) break;
        n += w;
    }
    if (n >= (int)capacity) n = (int)capacity - 1;
    return n;
}
