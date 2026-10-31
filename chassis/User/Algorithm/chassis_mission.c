#include "chassis_mission.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#define PI_F 3.14159265358979323846f
/* Tolerate a few failed IMU reads (one I2C timeout is 20 ms) before faulting. */
#define MISSION_IMU_DROPOUT_SEC 0.05f
/* Line counts as off-centre once it is at least half a probe pitch away. */
#define MISSION_SEARCH_SIDE_FRACTION 0.5f
#define MISSION_SEARCH_W_RAD_S 1.0f
/* Body yaw keeps turning for roughly this long after the command drops; the turn
 * brakes against the angle predicted that far ahead. Field traces stopped at 105-112 deg. */
#ifndef MISSION_TURN_LAG_SEC
#define MISSION_TURN_LAG_SEC 0.20f
#endif
#define MISSION_TURN_FAIL_RAD (130.0f*PI_F/180.0f)
/* Leave REACQUIRE once the line is within this many probe pitches of centre. */
#define MISSION_REACQUIRE_EXIT_PITCH 1.5f
/* ...and the body is within this angle of the new edge direction. */
#define MISSION_REACQUIRE_EXIT_RAD (15.0f*PI_F/180.0f)
/* ...and the body has nearly stopped turning (field logs: exits at ~0.7 rad/s). */
#define MISSION_REACQUIRE_EXIT_RATE 0.35f
/* Heading reference follows the gyro heading with this time constant while the line
 * is centred (gyro read 86-88 deg per real 90 deg corner in the field). */
#define MISSION_HEADING_TRIM_TAU_SEC 2.0f
/* Extra distance held at turn speed before a predicted corner, on top of the body-lag
 * cover, for the edge-length estimate (a fixed 0.15 m zone cost ~5.5 s per lap). */
#define MISSION_SLOW_MARGIN_M 0.04f
#define MISSION_SPEED_TAU_SEC 0.03f
/* Largest yaw-rate correction added to the wheel differential (rad/s): a gyro glitch cannot
 * spin the car. Field corrections are ~0.2-0.4 rad/s. */
#define MISSION_YAW_CORRECTION_MAX 1.0f

static float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float approach(float value, float goal, float step) { return value + clamp(goal-value,-step,step); }
static uint8_t count_bits(uint8_t mask) {
    uint8_t n=0; while (mask) { n+=(mask&1U); mask>>=1; } return n;
}
uint8_t mission_line_error(uint8_t mask, float spacing, float *error) {
    unsigned i, first=8, last=0, n=0; float sum=0;
    for (i=0;i<8;i++) if (mask & (1U<<i)) {
        if (first==8) first=i;
        last=i; n++; sum+=((float)i-3.5f)*spacing;
    }
    if (!n || n>3 || last-first+1!=n) return 0;
    *error=sum/(float)n; return 1;
}
static uint8_t corner(uint8_t mask) {
    return count_bits(mask&0x0fU)>=3 && (mask&0x18U)!=0 &&
           (mask&0x0cU)==0x0cU;
}
static void enter(chassis_mission_t *m, mission_state_t state) {
    m->state=state; m->state_time=0; m->state_distance=0;
    m->corner_confirm=m->center_confirm=0;
}
uint8_t mission_running(const chassis_mission_t *m) {
    return m->state>=MISSION_LINE && m->state<=MISSION_FINISH;
}
uint8_t mission_hold_vision(const chassis_mission_t *m) {
    return m->state==MISSION_PRE_TURN || m->state==MISSION_TURN ||
           m->state==MISSION_REACQUIRE || m->state==MISSION_FAULT;
}
float mission_wheel_w(const chassis_mission_t *m, float gyro_z_rad_s, uint8_t gyro_valid, float kg) {
    float correction;
    if (!gyro_valid || !isfinite(gyro_z_rad_s) || !isfinite(kg) || kg<=0) return m->w;
    if (m->state!=MISSION_LINE && m->state!=MISSION_PRE_TURN && m->state!=MISSION_FINISH) return m->w;
    correction=clamp(kg*(m->w-gyro_z_rad_s),-MISSION_YAW_CORRECTION_MAX,MISSION_YAW_CORRECTION_MAX);
    return m->w+correction;
}
void mission_init(chassis_mission_t *m, const mission_config_t *cfg) {
    memset(m,0,sizeof(*m)); m->cfg=*cfg;
}
void mission_stop(chassis_mission_t *m) {
    mission_config_t cfg=m->cfg; mission_init(m,&cfg);
}
void mission_fail(chassis_mission_t *m, mission_fault_t fault) {
    m->state=MISSION_FAULT; m->fault=fault; m->v=m->w=0;
}
typedef struct { const char *name; unsigned offset; float min, max; const char *format; } mission_param_t;
#define MISSION_PARAM(name, field, lo, hi, fmt) {name, offsetof(mission_config_t, field), lo, hi, fmt}
static const mission_param_t mission_params[] = {
    MISSION_PARAM("kg", yaw_rate_kg, 0.0f, 5.0f, "%.2f"),
    MISSION_PARAM("kp", line_kp, 0.0f, 0.1f, "%.4f"),
    MISSION_PARAM("kd", line_kd, 0.0f, 0.02f, "%.4f"),
    MISSION_PARAM("kh", line_kh, 0.0f, 5.0f, "%.2f"),
    MISSION_PARAM("accel", acceleration, 0.1f, 3.0f, "%.2f"),
    MISSION_PARAM("decel", deceleration, 0.1f, 3.0f, "%.2f"),
    MISSION_PARAM("wacc", line_angular_acceleration, 0.0f, 20.0f, "%.1f"),
    MISSION_PARAM("tacc", angular_acceleration, 1.0f, 20.0f, "%.1f"),
    MISSION_PARAM("turn", turn_m_s, MISSION_TURN_MIN_M_S, 0.5f, "%.3f"),
    MISSION_PARAM("cruise", cruise_m_s, 0.1f, 0.5f, "%.3f"),
    MISSION_PARAM("edge", edge_m, 0.6f, 1.4f, "%.2f"),
    MISSION_PARAM("circle", circle, 0.0f, 1.0f, "%.0f"),
};
#define MISSION_PARAM_COUNT (sizeof(mission_params)/sizeof(mission_params[0]))
static float *param_field(mission_config_t *c, const mission_param_t *p) {
    return (float *)((char *)c+p->offset);
}
static float param_value(const mission_config_t *c, const mission_param_t *p) {
    return *(const float *)((const char *)c+p->offset);
}
/* The command line upper-cases everything, so names match regardless of case. */
static uint8_t name_equal(const char *a, const char *b) {
    for (;;a++,b++) {
        const char x=(char)((*a>='A' && *a<='Z') ? *a-'A'+'a' : *a);
        const char y=(char)((*b>='A' && *b<='Z') ? *b-'A'+'a' : *b);
        if (x!=y) return 0;
        if (!x) return 1;
    }
}
uint8_t mission_set_param(chassis_mission_t *m, const char *name, float value) {
    for (unsigned i=0;i<MISSION_PARAM_COUNT;i++) {
        const mission_param_t *p=&mission_params[i];
        mission_config_t c;
        if (!name_equal(name,p->name)) continue;
        if (mission_running(m) || !isfinite(value) || value<p->min || value>p->max) return 0;
        c=m->cfg; *param_field(&c,p)=value;
        if (c.turn_m_s>c.cruise_m_s || c.first_edge_m_s>c.cruise_m_s ||
            c.sensor_ahead_m<c.radius_m+c.turn_m_s*0.06f) return 0;
        m->cfg=c;
        return 1;
    }
    return 0;
}
uint8_t mission_set_turn_speed(chassis_mission_t *m, float turn_m_s) {
    return mission_set_param(m,"turn",turn_m_s);
}
int mission_format_params(const chassis_mission_t *m, char *text, unsigned cap) {
    unsigned used=0;
    if (!cap) return 0;
    text[0]=0;
    for (unsigned i=0;i<MISSION_PARAM_COUNT;i++) {
        const mission_param_t *p=&mission_params[i];
        char value[16];
        int n=snprintf(value,sizeof(value),p->format,(double)param_value(&m->cfg,p));
        if (n<0) break;
        n=snprintf(text+used,cap-used,"%s%s=%s",used?" ":"",p->name,value);
        if (n<0 || (unsigned)n>=cap-used) break;
        used+=(unsigned)n;
    }
    return (int)used;
}
uint8_t mission_start(chassis_mission_t *m, uint8_t laps) {
    const mission_config_t *c=&m->cfg;
    if (mission_running(m) || m->state==MISSION_FAULT) return 0;
    if (laps<1 || laps>5 || !isfinite(c->wheelbase_m) || !isfinite(c->sensor_ahead_m) ||
        !isfinite(c->probe_spacing_mm) || !isfinite(c->radius_m) || !isfinite(c->cruise_m_s) ||
        !isfinite(c->turn_m_s) || !isfinite(c->acceleration) || !isfinite(c->deceleration) ||
        !isfinite(c->angular_acceleration) || !isfinite(c->line_kp) || !isfinite(c->line_kd) ||
        !isfinite(c->min_edge_m) || !isfinite(c->max_edge_m) || !isfinite(c->line_kh) || c->line_kh<0 ||
        !isfinite(c->edge_m) || c->edge_m<0 ||
        !isfinite(c->first_edge_m_s) || c->first_edge_m_s<0 || c->first_edge_m_s>c->cruise_m_s ||
        c->wheelbase_m<0.05f || c->wheelbase_m>0.4f || c->radius_m<=c->wheelbase_m*0.5f ||
        c->sensor_ahead_m<c->radius_m+c->turn_m_s*0.06f || c->sensor_ahead_m>0.3f ||
        c->probe_spacing_mm<1 || c->probe_spacing_mm>30 || c->turn_m_s<0.05f ||
        c->cruise_m_s<c->turn_m_s || c->cruise_m_s>0.5f ||
        c->acceleration<=0 || c->deceleration<=0 || c->angular_acceleration<=0 ||
        c->line_kp<0 || c->line_kd<0 || c->min_edge_m<0.2f || c->max_edge_m<=c->min_edge_m) {
        mission_fail(m,MISSION_BAD_CONFIG); return 0;
    }
    mission_stop(m); m->laps=laps; enter(m,MISSION_LINE); return 1;
}

/* Gyro if the sample is usable; otherwise the encoder yaw rate for a short dropout.
 * Returns 0 once the dropout budget is exhausted. */
static uint8_t yaw_rate(chassis_mission_t *m, const mission_input_t *in, float *rate) {
    if (in->imu_valid && isfinite(in->gyro_z_rad_s) && fabsf(in->gyro_z_rad_s)<=10) {
        m->imu_invalid_time=0; *rate=in->gyro_z_rad_s; return 1;
    }
    m->imu_invalid_time+=in->dt;
    if (m->imu_invalid_time>MISSION_IMU_DROPOUT_SEC) return 0;
    *rate=(in->right_delta_m-in->left_delta_m)/(m->cfg.wheelbase_m*in->dt);
    return 1;
}

/* No probe sees the line: turn toward the side it was last seen on (error left
 * negative -> counter-clockwise, positive w). A centred line means a gap: go straight. */
static float search_w(float last_error_mm, float spacing_mm) {
    const float side=MISSION_SEARCH_SIDE_FRACTION*spacing_mm;
    if (last_error_mm<=-side) return MISSION_SEARCH_W_RAD_S;
    if (last_error_mm>=side) return -MISSION_SEARCH_W_RAD_S;
    return 0;
}

float mission_heading_error(const chassis_mission_t *m, unsigned edge) {
    return m->heading-(float)edge*PI_F*0.5f-m->heading_trim;
}

/* Trigger-to-trigger spacing expected for the next corner: measured average once
 * an edge has been seen, otherwise the configured nominal edge. */
static float next_edge_m(const chassis_mission_t *m) {
    return (m->corners>=2) ? m->edge_sum/(float)(m->corners-1) : m->cfg.edge_m;
}
float mission_run_progress(const chassis_mission_t *m) {
    const float laps=(float)m->laps;
    float p, lap;
    if (!m->laps) return 0.0f;
    if (m->state==MISSION_DONE) return laps;
    if (m->state==MISSION_FINISH && m->finish_distance>0.0f) p=laps*m->distance/m->finish_distance;
    else {
        /* A corner's edge is added to edge_sum at its trigger, the corner count only after the
         * turn: count the triggers seen so far (next_edge_m is only used on the straights). */
        const unsigned triggers=m->corners+((m->state==MISSION_PRE_TURN || m->state==MISSION_TURN ||
                                             m->state==MISSION_REACQUIRE) ? 1U : 0U);
        lap=4.0f*(triggers>=2U ? m->edge_sum/(float)(triggers-1U) : m->cfg.edge_m);
        if (!(lap>0.0f)) return 0.0f;
        p=m->distance/lap;
    }
    return p<0.0f ? 0.0f : p>laps ? laps : p;
}
/* Distance the body keeps rolling straight before it actually starts to yaw. */
static float turn_lead_m(const chassis_mission_t *m) {
    const float w_turn=m->cfg.turn_m_s/m->cfg.radius_m;
    return m->v_meas*(MISSION_TURN_LAG_SEC+w_turn/(2.0f*m->cfg.angular_acceleration));
}

void mission_step(chassis_mission_t *m, const mission_input_t *in) {
    float ds, error=0, target_v, target_w, remaining, gyro_z, new_edge_error;
    uint8_t normal;
    if (!mission_running(m)) return;
    if (!isfinite(in->dt) || in->dt<0.005f || in->dt>0.030f) {
        mission_fail(m,MISSION_LATE); return;
    }
    if (!in->line_valid || !isfinite(in->left_delta_m) || !isfinite(in->right_delta_m) ||
        fabsf(in->left_delta_m)>0.8f*in->dt+0.001f || fabsf(in->right_delta_m)>0.8f*in->dt+0.001f ||
        !yaw_rate(m,in,&gyro_z)) {
        mission_fail(m,MISSION_SENSOR); return;
    }
    m->heading+=gyro_z*in->dt;
    ds=(in->left_delta_m+in->right_delta_m)*0.5f;
    m->v_meas+=in->dt/(MISSION_SPEED_TAU_SEC+in->dt)*(ds/in->dt-m->v_meas);
    m->distance+=ds; m->state_distance+=ds; m->edge_distance+=ds;
    m->elapsed+=in->dt; m->state_time+=in->dt;
    if (m->elapsed>40.0f*m->laps+5.0f) { mission_fail(m,MISSION_ROUTE); return; }
    m->left_stall=(m->v-m->w*m->cfg.wheelbase_m*0.5f>0.025f && fabsf(in->left_delta_m)<1e-7f) ? m->left_stall+in->dt : 0;
    m->right_stall=(m->v+m->w*m->cfg.wheelbase_m*0.5f>0.025f && fabsf(in->right_delta_m)<1e-7f) ? m->right_stall+in->dt : 0;
    if (m->left_stall>0.8f || m->right_stall>0.8f) { mission_fail(m,MISSION_STALL); return; }
    normal=mission_line_error(in->line_mask,m->cfg.probe_spacing_mm,&error);
    target_v=m->cfg.cruise_m_s; target_w=0;

    if (m->state==MISSION_LINE || m->state==MISSION_FINISH) {
        if (m->state==MISSION_LINE && in->new_line) {
            if (corner(in->line_mask) && (m->corners==0 || m->edge_distance>=m->cfg.min_edge_m)) {
                if (m->corner_confirm<3) m->corner_confirm++;
            } else m->corner_confirm=0;
            if (m->corner_confirm>=3) {
                if (!m->corners) m->first_trigger=m->distance;
                else m->edge_sum+=m->distance-m->last_trigger;
                m->last_trigger=m->distance;
                enter(m,MISSION_PRE_TURN); m->lost_time=0;
            }
        }
        if (m->state==MISSION_LINE && m->edge_distance>m->cfg.max_edge_m) {
            mission_fail(m,MISSION_ROUTE); return;
        }
        if (normal) {
            float previous=m->line_error;
            if (!m->error_initialized) { m->line_error=error; previous=error; m->error_initialized=1; }
            else m->line_error+=in->dt/(0.02f+in->dt)*(error-m->line_error);
            const float heading_error=mission_heading_error(m,m->corners);
            m->derivative+=in->dt/(0.03f+in->dt)*((m->line_error-previous)/in->dt-m->derivative);
            target_w=clamp(-m->cfg.line_kp*m->line_error-m->cfg.line_kd*m->derivative-
                           m->cfg.line_kh*heading_error,-1.5f,1.5f);
            /* Line centred: the line is the long-term truth, so let the heading
             * reference follow the gyro heading (absorbs gyro scale error and drift). */
            if (fabsf(m->line_error)<0.5f*m->cfg.probe_spacing_mm)
                m->heading_trim+=in->dt/(MISSION_HEADING_TRIM_TAU_SEC+in->dt)*heading_error;
            target_v*=1.0f-0.3f*clamp(fabsf(m->line_error)/40.0f,0,1);
            m->lost_time=0;
        } else {
            m->lost_time+=in->dt; target_v=0.08f; target_w=m->w;
            if (!in->line_mask && m->error_initialized)
                target_w=search_w(m->line_error,m->cfg.probe_spacing_mm);
            if (m->lost_time>0.20f) { mission_fail(m,MISSION_LOST_LINE); return; }
        }
        if (m->state==MISSION_LINE && m->corners) {
            /* Brake as late as the deceleration allows, arriving at turn speed with a
             * short hold for the body lag and the edge-length estimate. */
            const float expected=next_edge_m(m);
            if (expected>0) {
                const float hold=m->cfg.turn_m_s*MISSION_TURN_LAG_SEC+MISSION_SLOW_MARGIN_M;
                const float room=fmaxf(0,expected-(m->distance-m->last_trigger)-hold);
                target_v=fminf(target_v,sqrtf(m->cfg.turn_m_s*m->cfg.turn_m_s+
                                              2.0f*m->cfg.deceleration*room));
            }
        } else if (m->state==MISSION_LINE && m->cfg.first_edge_m_s>0) {
            target_v=fminf(target_v,m->cfg.first_edge_m_s);
        }
        if (m->state==MISSION_FINISH) {
            remaining=m->finish_distance-m->distance;
            if (remaining<=0.003f) { enter(m,MISSION_DONE); m->v=m->w=0; return; }
            target_v=fminf(target_v,sqrtf(2.0f*m->cfg.deceleration*remaining));
        }
    }
    if (m->state==MISSION_PRE_TURN) {
        /* Broadcast hold before yaw motion; the forward travel is part of this state. */
        target_w=0; target_v=m->cfg.turn_m_s;
        remaining=m->cfg.sensor_ahead_m-m->cfg.radius_m-turn_lead_m(m)-m->state_distance;
        if (remaining<=0 && m->state_time>=0.06f && fabsf(m->w)<0.05f) {
            /* Forget the pre-corner line position; REACQUIRE learns the new one. */
            enter(m,MISSION_TURN); m->turn_angle=0; m->error_initialized=0;
        }
        if (m->state_time>2.0f) { mission_fail(m,MISSION_TURN_TIMEOUT); return; }
    }
    if (m->state==MISSION_TURN || m->state==MISSION_REACQUIRE) {
        m->turn_angle+=gyro_z*in->dt;
        target_v=m->cfg.turn_m_s;
        remaining=fmaxf(0,PI_F*0.5f-m->turn_angle-gyro_z*MISSION_TURN_LAG_SEC);
        target_w=fminf(fmaxf(m->v_meas,m->cfg.turn_m_s)/m->cfg.radius_m,
                       sqrtf(2.0f*m->cfg.angular_acceleration*remaining));
        if (m->turn_angle>MISSION_TURN_FAIL_RAD || m->turn_angle< -15.0f*PI_F/180.0f || m->state_time>5.0f) {
            mission_fail(m,MISSION_TURN_TIMEOUT); return;
        }
        if (m->turn_angle>=80.0f*PI_F/180.0f) m->state=MISSION_REACQUIRE; /* Keep turn timeout running. */
        /* Heading error against the edge this turn leads onto (+ = over-rotated). */
        new_edge_error=mission_heading_error(m,m->corners+1U);
        if (m->state==MISSION_REACQUIRE) {
            /* The body may land beside the new line: steer onto it wherever it shows,
             * while pulling the heading back to the edge so it does not over-rotate. */
            if (normal) {
                if (!m->error_initialized) { m->line_error=error; m->error_initialized=1; }
                else m->line_error+=in->dt/(0.02f+in->dt)*(error-m->line_error);
                target_w=clamp(-m->cfg.line_kp*m->line_error-m->cfg.line_kh*new_edge_error,-1.5f,1.5f);
            } else if (!in->line_mask && m->error_initialized) {
                target_w=search_w(m->line_error,m->cfg.probe_spacing_mm);
            } else if (new_edge_error>0) {
                target_w=clamp(-m->cfg.line_kh*new_edge_error,-1.5f,1.5f);
            }
        }
        if (in->new_line) {
            if (m->state==MISSION_REACQUIRE && normal &&
                fabsf(error)<=MISSION_REACQUIRE_EXIT_PITCH*m->cfg.probe_spacing_mm &&
                fabsf(new_edge_error)<=MISSION_REACQUIRE_EXIT_RAD &&
                fabsf(gyro_z)<MISSION_REACQUIRE_EXIT_RATE) {
                if (m->center_confirm<3) m->center_confirm++;
            } else m->center_confirm=0;
        }
        if (m->center_confirm>=3) {
            m->corners++; m->edge_distance=0; m->error_initialized=0; m->lost_time=0; m->derivative=0;
            if (m->corners>=4*m->laps) {
                /* Use trigger-to-trigger distances, including arcs, consistently. */
                m->finish_distance=m->last_trigger+m->edge_sum/(float)(m->corners-1)-m->first_trigger;
                /* Start already passed or too close to brake: stop as soon as possible. */
                if (m->finish_distance<=m->distance) { enter(m,MISSION_DONE); m->v=m->w=0; return; }
                enter(m,MISSION_FINISH);
            } else enter(m,MISSION_LINE);
            target_w=0;
        }
    }
    m->v=approach(m->v,target_v,(target_v>m->v?m->cfg.acceleration:m->cfg.deceleration)*in->dt);
    /* Straights may use a gentler w slew: line-error steps (one probe = 11 mm) then do not jerk
     * the body (field 2026-10-28: 12-31 deg/s yaw kicks on straights doubled the gimbal error). */
    const float w_acc=((m->state==MISSION_LINE || m->state==MISSION_PRE_TURN || m->state==MISSION_FINISH) &&
                       m->cfg.line_angular_acceleration>0) ? m->cfg.line_angular_acceleration
                                                          : m->cfg.angular_acceleration;
    m->w=approach(m->w,target_w,w_acc*in->dt);
}
