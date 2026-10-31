#include "chassis_mission.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define PI_TEST 3.14159265f

static mission_config_t cfg={0.122f,0.12f,12,0.1f,0.2f,0.12f,0.5f,0.8f,3,0.02f,0.002f,0.45f,1.5f,1.5f,0.9f,0.0f,0.0f,0.0f,0.0f};
static void tick(chassis_mission_t *m, uint8_t mask, uint8_t fresh) {
    mission_input_t in={0};
    in.dt=0.01f; in.line_mask=mask; in.line_valid=in.imu_valid=1; in.new_line=fresh;
    in.left_delta_m=(m->v-m->w*cfg.wheelbase_m*0.5f)*in.dt;
    in.right_delta_m=(m->v+m->w*cfg.wheelbase_m*0.5f)*in.dt;
    in.gyro_z_rad_s=m->w;
    mission_step(m,&in);
}
static void test_line_and_faults(void) {
    chassis_mission_t m; float e=99;
    assert(mission_line_error(0x18,12,&e) && e==0);
    assert(mission_line_error(0x60,12,&e) && e>0);
    assert(!mission_line_error(0x81,12,&e));
    assert(!mission_line_error(0,12,&e));
    assert(!mission_line_error(0xff,12,&e));
    mission_init(&m,&cfg); assert(!mission_running(&m));
    assert(mission_start(&m,1));
    tick(&m,0x60,1); assert(m.w<0); /* right line -> clockwise */
    for (int i=0;i<25;i++) tick(&m,0,1);
    assert(m.state==MISSION_FAULT && m.fault==MISSION_LOST_LINE && m.v==0 && m.w==0);
    assert(!mission_start(&m,1)); /* fault cannot self-restart */
    mission_stop(&m); assert(mission_start(&m,1));
    mission_input_t in={0}; in.dt=0.04f;
    mission_step(&m,&in); assert(m.fault==MISSION_LATE);
    mission_stop(&m); assert(mission_start(&m,1)); in.dt=0.01f;
    mission_step(&m,&in); assert(m.fault==MISSION_SENSOR);
    mission_stop(&m); assert(!mission_start(&m,6));
    mission_stop(&m); m.cfg.probe_spacing_mm=NAN; assert(!mission_start(&m,1));
}
static void test_confirmation_and_preturn(void) {
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    tick(&m,0x0f,1);
    for (int i=0;i<10;i++) tick(&m,0x0f,0);
    assert(m.state==MISSION_LINE); /* do not count one scan repeatedly */
    tick(&m,0x0f,1); tick(&m,0x0f,1);
    assert(m.state==MISSION_PRE_TURN && mission_hold_vision(&m));
    for (int i=0;i<4;i++) { tick(&m,0x18,1); assert(m.state==MISSION_PRE_TURN && m.w==0); }
    mission_stop(&m); assert(m.state==MISSION_IDLE && m.v==0 && m.w==0);
}
static void test_route_from(unsigned laps, float first_corner_m) {
    chassis_mission_t m; unsigned steps, saw_hold=0, returns=0;
    mission_init(&m,&cfg); assert(mission_start(&m,(uint8_t)laps));
    for (steps=0;steps<25000 && mission_running(&m);steps++) {
        uint8_t mask=0x18;
        mission_state_t previous=m.state;
        if (m.state==MISSION_LINE && ((!m.corners && m.distance>=first_corner_m) ||
            (m.corners && m.edge_distance>=0.70f))) mask=0x0f;
        if ((m.state==MISSION_TURN || m.state==MISSION_REACQUIRE) && m.turn_angle<1.48f) mask=0;
        if (mission_hold_vision(&m)) saw_hold++;
        tick(&m,mask,1);
        if (previous==MISSION_REACQUIRE && (m.state==MISSION_LINE || m.state==MISSION_FINISH)) returns++;
    }
    if (first_corner_m>0.8f) { /* start just after a corner: stop as soon as possible */
        assert(m.state==MISSION_DONE && m.corners==4*laps && m.v==0 && m.w==0);
        return;
    }
    if (m.state!=MISSION_DONE) printf("route fail: state=%d fault=%d corner=%d angle=%f distance=%f\n",m.state,m.fault,m.corners,m.turn_angle,m.distance);
    assert(m.state==MISSION_DONE && m.corners==4*laps && saw_hold && returns==4*laps);
    assert(m.v==0 && m.w==0 && fabsf(m.distance-m.finish_distance)<0.01f);
    for (unsigned i=0;i<100;i++) tick(&m,0x18,1);
    assert(m.state==MISSION_DONE); /* no automatic lap restart */
}
static void test_route(unsigned laps) { test_route_from(laps,0.4f); }
/* Run progress in laps (the gimbal draws one circle per lap from it): 0 before the start,
 * exactly `laps` at DONE, never above, no jumps, and within 0.06 lap of the true share of the
 * run (distance / final finish distance; the task allows 1/4 lap) all the way. */
static void test_run_progress(unsigned laps, float first_corner_m) {
    static float distance[25000], progress[25000];
    chassis_mission_t m; unsigned n=0;
    mission_init(&m,&cfg); assert(mission_run_progress(&m)==0.0f);
    assert(mission_start(&m,(uint8_t)laps)); assert(mission_run_progress(&m)==0.0f);
    while (n<25000 && mission_running(&m)) {
        uint8_t mask=0x18;
        if (m.state==MISSION_LINE && ((!m.corners && m.distance>=first_corner_m) ||
            (m.corners && m.edge_distance>=0.70f))) mask=0x0f;
        if ((m.state==MISSION_TURN || m.state==MISSION_REACQUIRE) && m.turn_angle<1.48f) mask=0;
        tick(&m,mask,1);
        distance[n]=m.distance; progress[n]=mission_run_progress(&m); n++;
    }
    assert(m.state==MISSION_DONE && progress[n-1]==(float)laps);
    for (unsigned i=0;i<n;i++) {
        const float truth=(float)laps*distance[i]/m.finish_distance;
        assert(progress[i]>=0.0f && progress[i]<=(float)laps);
        assert(fabsf(progress[i]-(truth<(float)laps?truth:(float)laps))<0.06f);
        if (i) assert(fabsf(progress[i]-progress[i-1])<0.03f);
    }
    mission_stop(&m); assert(mission_run_progress(&m)==0.0f);
}
static void test_slow_before_known_corner(void) {
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    while (m.distance<0.4f) tick(&m,0x18,1);
    for (int i=0;i<3;i++) tick(&m,0x0f,1);
    for (int i=0;i<800 && !(m.state==MISSION_LINE && m.corners==1);i++)
        tick(&m,(m.state==MISSION_TURN || (m.state==MISSION_REACQUIRE && m.turn_angle<1.48f)) ? 0 : 0x18,1);
    assert(m.state==MISSION_LINE && m.corners==1);
    while (m.distance-m.last_trigger<cfg.edge_m-0.35f) tick(&m,0x18,1);
    assert(m.v>cfg.turn_m_s+0.05f);          /* full speed away from the corner */
    while (m.distance-m.last_trigger<cfg.edge_m-0.03f) tick(&m,0x18,1);
    assert(m.v<=cfg.turn_m_s+0.01f);         /* already slow when the next corner shows */
}
static void drive_to_first_edge_exit(chassis_mission_t *m) {
    while (m->distance<0.4f) tick(m,0x18,1);
    for (int i=0;i<3;i++) tick(m,0x0f,1);
    for (int i=0;i<800 && !(m->state==MISSION_LINE && m->corners==1);i++)
        tick(m,(m->state==MISSION_TURN || (m->state==MISSION_REACQUIRE && m->turn_angle<1.48f)) ? 0 : 0x18,1);
    assert(m->state==MISSION_LINE && m->corners==1);
}
static void test_braking_starts_at_the_braking_distance(void) {
    /* A fixed 0.15 m slow zone cost ~5.5 s per lap at 0.10 m/s. Brake only as late as
     * the deceleration and the body lag allow, then hold turn speed briefly. */
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    drive_to_first_edge_exit(&m);
    while (m.distance-m.last_trigger<cfg.edge_m-0.11f) tick(&m,0x18,1);
    assert(m.v>cfg.turn_m_s+0.06f);          /* still near cruise 0.11 m before */
    while (m.distance-m.last_trigger<cfg.edge_m-0.03f) tick(&m,0x18,1);
    assert(m.v<=cfg.turn_m_s+0.01f);         /* at turn speed when the corner shows */
}
static void test_first_edge_is_capped(void) {
    /* Corner 1 position is unknown (start anywhere on an edge), so it is entered at full
     * speed; at 0.30 m/s the model lands it 20-42 mm outside. Cap the first edge. */
    mission_config_t capped=cfg; chassis_mission_t m;
    capped.first_edge_m_s=0.15f;
    mission_init(&m,&capped); assert(mission_start(&m,1));
    for (int i=0;i<150;i++) { tick(&m,0x18,1); assert(m.v<=0.15f+1e-4f); }
    drive_to_first_edge_exit(&m);
    for (int i=0;i<80;i++) tick(&m,0x18,1);
    assert(m.v>0.18f);                         /* full cruise once corner 1 is behind */
}
static void test_turn_rate_uses_measured_speed(void) {
    chassis_mission_t m; mission_input_t in={0};
    mission_init(&m,&cfg); assert(mission_start(&m,1));
    for (int i=0;i<3;i++) tick(&m,0x0f,1);
    for (int i=0;i<100 && m.state!=MISSION_TURN;i++) tick(&m,0x18,1);
    assert(m.state==MISSION_TURN);
    in.dt=0.01f; in.line_valid=in.imu_valid=in.new_line=1; in.line_mask=0;
    for (int i=0;i<60;i++) { /* body still rolling at 0.2 m/s although 0.12 is commanded */
        in.left_delta_m=(0.2f-m.w*cfg.wheelbase_m*0.5f)*in.dt;
        in.right_delta_m=(0.2f+m.w*cfg.wheelbase_m*0.5f)*in.dt;
        in.gyro_z_rad_s=0.02f; mission_step(&m,&in);
    }
    assert(m.state==MISSION_TURN && m.w>cfg.turn_m_s/cfg.radius_m+0.2f);
}
static void test_stall_and_turn_timeout(void) {
    chassis_mission_t m; mission_input_t in={0};
    mission_init(&m,&cfg); assert(mission_start(&m,1));
    in.dt=0.01f; in.line_valid=in.imu_valid=in.new_line=1; in.line_mask=0x18;
    for (int i=0;i<150;i++) mission_step(&m,&in);
    assert(m.fault==MISSION_STALL);
    mission_stop(&m); assert(mission_start(&m,1));
    for (int i=0;i<3;i++) tick(&m,0x0f,1);
    for (int i=0;i<1000 && mission_running(&m);i++) {
        in.left_delta_m=in.right_delta_m=0.0012f; in.gyro_z_rad_s=0;
        mission_step(&m,&in);
    }
    assert(m.fault==MISSION_TURN_TIMEOUT);
}
static void tick_imu(chassis_mission_t *m, uint8_t mask, uint8_t imu_valid) {
    mission_input_t in={0};
    in.dt=0.01f; in.line_mask=mask; in.line_valid=1; in.imu_valid=imu_valid; in.new_line=1;
    in.left_delta_m=(m->v-m->w*cfg.wheelbase_m*0.5f)*in.dt;
    in.right_delta_m=(m->v+m->w*cfg.wheelbase_m*0.5f)*in.dt;
    in.gyro_z_rad_s=imu_valid ? m->w : 0.0f;
    mission_step(m,&in);
}
static void test_imu_dropout_tolerance(void) {
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    for (int i=0;i<20;i++) tick(&m,0x18,1);
    for (int i=0;i<4;i++) { tick_imu(&m,0x18,0); assert(m.state==MISSION_LINE); }
    tick(&m,0x18,1); /* one valid sample re-arms the dropout budget */
    for (int i=0;i<4;i++) { tick_imu(&m,0x18,0); assert(m.state==MISSION_LINE); }
    for (int i=0;i<3;i++) tick_imu(&m,0x18,0);
    assert(m.state==MISSION_FAULT && m.fault==MISSION_SENSOR);

    /* During a turn the encoder yaw rate keeps the turn angle advancing. */
    mission_stop(&m); assert(mission_start(&m,1));
    for (int i=0;i<3;i++) tick(&m,0x0f,1);
    for (int i=0;i<200 && m.state!=MISSION_TURN;i++) tick(&m,0x18,1);
    assert(m.state==MISSION_TURN);
    for (int i=0;i<20;i++) tick(&m,0,1);
    {
        const float before=m.turn_angle;
        for (int i=0;i<3;i++) tick_imu(&m,0,0);
        assert(m.state==MISSION_TURN && m.turn_angle>before+0.01f);
    }
}
static float follow_then_lose(uint8_t mask, float *w_before) {
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    for (int i=0;i<30;i++) tick(&m,mask,1);
    *w_before=m.w;
    for (int i=0;i<10;i++) tick(&m,0,1);
    assert(m.state==MISSION_LINE); /* still within the lost-line budget */
    return m.w;
}
static void test_lost_line_search_direction(void) {
    float before, after;
    after=follow_then_lose(0x0c,&before); /* line last seen on the left */
    assert(before>0 && after>before+0.1f);
    after=follow_then_lose(0x30,&before); /* line last seen on the right */
    assert(before<0 && after<before-0.1f);
    after=follow_then_lose(0x18,&before); /* centred: keep straight across a gap */
    assert(fabsf(after)<0.05f);
}
/* Body yaw rate follows the command with a first-order lag, like the real drivetrain. */
static float plant_w;
static void tick_lag(chassis_mission_t *m, uint8_t mask, float lag_sec) {
    mission_input_t in={0};
    in.dt=0.01f; in.line_mask=mask; in.line_valid=in.imu_valid=in.new_line=1;
    plant_w+=(m->w-plant_w)*in.dt/lag_sec;
    in.left_delta_m=(m->v-plant_w*cfg.wheelbase_m*0.5f)*in.dt;
    in.right_delta_m=(m->v+plant_w*cfg.wheelbase_m*0.5f)*in.dt;
    in.gyro_z_rad_s=plant_w;
    mission_step(m,&in);
}
static void test_turn_overshoot_with_lag(void) {
    chassis_mission_t m; float peak=0;
    mission_init(&m,&cfg); assert(mission_start(&m,1)); plant_w=0;
    while (m.distance<0.3f) tick_lag(&m,0x18,0.2f);
    for (int i=0;i<3;i++) tick_lag(&m,0x0f,0.2f);
    assert(m.state==MISSION_PRE_TURN);
    for (int i=0;i<300 && mission_running(&m);i++) {
        tick_lag(&m,0,0.2f);
        if (m.turn_angle>peak) peak=m.turn_angle;
    }
    fprintf(stderr,"lagged turn peak %.1f deg\n",peak*57.29578f);
    assert(m.state==MISSION_REACQUIRE); /* no over-rotation fault */
    assert(peak<95.0f*PI_TEST/180.0f && peak>85.0f*PI_TEST/180.0f);
}
static void test_heading_hold(void) {
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    for (int i=0;i<20;i++) tick(&m,0x18,1);
    m.heading=0.3f; /* body rotated 17 deg left of the edge, line still centred */
    for (int i=0;i<10;i++) tick(&m,0x18,1);
    assert(m.state==MISSION_LINE && m.w<-0.2f);
}
static void test_reacquire_offset_line(void) {
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    for (int i=0;i<3;i++) tick(&m,0x0f,1);
    for (int i=0;i<400 && m.state!=MISSION_REACQUIRE;i++) tick(&m,0,1);
    assert(m.state==MISSION_REACQUIRE);
    m.turn_angle=1.65f; /* already past 90 deg, line seen on the far left */
    for (int i=0;i<15;i++) tick(&m,0x03,1);
    assert(m.state==MISSION_REACQUIRE && m.w>0.4f); /* steer toward it, not straight */
    m.turn_angle=1.92f; /* 110 deg while converging must not fault */
    tick(&m,0x03,1);
    assert(m.state==MISSION_REACQUIRE);
    m.heading=1.5708f+0.5f; /* line centred but body 29 deg over-rotated: keep correcting */
    {
        const float w_before=m.w;
        for (int i=0;i<5;i++) tick(&m,0x18,1);
        assert(m.state==MISSION_REACQUIRE && m.w<w_before-0.1f);
    }
    m.heading=1.5708f;
    for (int i=0;i<80 && m.state==MISSION_REACQUIRE;i++) tick(&m,0x18,1);
    assert(m.state==MISSION_LINE && m.corners==1);
}
static void test_reacquire_waits_for_rotation_to_stop(void) {
    /* Field logs: the probes swept onto the new line while the body still turned at
     * ~0.7 rad/s, and handing over then left a 5-10 deg transient. */
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    for (int i=0;i<3;i++) tick(&m,0x0f,1);
    for (int i=0;i<400 && m.state!=MISSION_REACQUIRE;i++) tick(&m,0,1);
    assert(m.state==MISSION_REACQUIRE);
    m.heading=1.5708f; m.w=0.7f;
    for (int i=0;i<4;i++) { tick(&m,0x18,1); assert(m.state==MISSION_REACQUIRE); }
    for (int i=0;i<80 && m.state==MISSION_REACQUIRE;i++) tick(&m,0x18,1);
    assert(m.state==MISSION_LINE && fabsf(m.w)<0.4f);
}
static void tick_gyro_bias(chassis_mission_t *m, uint8_t mask, float bias) {
    mission_input_t in={0};
    in.dt=0.01f; in.line_mask=mask; in.line_valid=in.imu_valid=in.new_line=1;
    in.left_delta_m=(m->v-m->w*cfg.wheelbase_m*0.5f)*in.dt;
    in.right_delta_m=(m->v+m->w*cfg.wheelbase_m*0.5f)*in.dt;
    in.gyro_z_rad_s=m->w+bias;
    mission_step(m,&in);
}
static void test_heading_reference_follows_a_centred_line(void) {
    /* Field log: gyro heading read 86-88 deg per corner while the line stayed centred,
     * so heading hold kept fighting the line controller. With the line centred for a
     * while, the reference must follow the measured heading instead of building up. */
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    for (int i=0;i<600 && m.state==MISSION_LINE;i++) tick_gyro_bias(&m,0x18,-0.02f); /* 1.2 m, < max edge */
    assert(m.state==MISSION_LINE);
    assert(fabsf(m.w)<0.06f);   /* without the trim: 1.5 * 0.12 rad = 0.18 rad/s */
}
static void test_wheel_yaw_rate_loop(void) {
    chassis_mission_t m; mission_init(&m,&cfg); assert(mission_start(&m,1));
    m.w=0.1f;                                         /* LINE: body lags +0.3 -> push back */
    assert(fabsf(mission_wheel_w(&m,0.3f,1,2.0f)-(0.1f+2.0f*(0.1f-0.3f)))<1e-6f);
    assert(fabsf(mission_wheel_w(&m,-5.0f,1,2.0f)-(0.1f+1.0f))<1e-6f);   /* clamped */
    assert(mission_wheel_w(&m,0.3f,0,2.0f)==0.1f);    /* gyro not valid */
    assert(mission_wheel_w(&m,NAN,1,2.0f)==0.1f);
    assert(mission_wheel_w(&m,0.3f,1,0.0f)==0.1f);    /* loop off */
    m.state=MISSION_TURN; m.w=2.0f;                   /* turns keep their tuned response */
    assert(mission_wheel_w(&m,1.6f,1,2.0f)==2.0f);
    m.state=MISSION_PRE_TURN; m.w=0.0f;
    assert(fabsf(mission_wheel_w(&m,0.2f,1,2.0f)+0.4f)<1e-6f);
}

static void test_set_turn_speed(void) {
    chassis_mission_t m;
    mission_init(&m,&cfg);
    assert(mission_set_turn_speed(&m,0.10f) && m.cfg.turn_m_s==0.10f);
    assert(!mission_set_turn_speed(&m,0.05f) && m.cfg.turn_m_s==0.10f);   /* below minimum */
    assert(!mission_set_turn_speed(&m,0.25f) && m.cfg.turn_m_s==0.10f);   /* above cruise */
    assert(!mission_set_turn_speed(&m,NAN) && m.cfg.turn_m_s==0.10f);
    mission_stop(&m); assert(m.cfg.turn_m_s==0.10f);                     /* kept across runs */
    assert(mission_start(&m,1));
    assert(!mission_set_turn_speed(&m,0.12f) && m.cfg.turn_m_s==0.10f);   /* not while running */
}
static void test_set_param_and_format(void) {
    chassis_mission_t m; char text[200];
    mission_init(&m,&cfg);
    assert(mission_set_param(&m,"KG",1.0f) && m.cfg.yaw_rate_kg==1.0f);       /* upper case, as received */
    assert(mission_set_param(&m,"decel",0.6f) && m.cfg.deceleration==0.6f);
    assert(mission_set_param(&m,"WACC",3.0f) && m.cfg.line_angular_acceleration==3.0f);
    assert(!mission_set_param(&m,"nope",1.0f));
    assert(!mission_set_param(&m,"kg",6.0f) && m.cfg.yaw_rate_kg==1.0f);        /* out of range */
    assert(!mission_set_param(&m,"cruise",0.11f) && m.cfg.cruise_m_s==0.2f);   /* below the turn speed */
    assert(mission_format_params(&m,text,sizeof(text))>0);
    assert(strstr(text,"kg=1.00") && strstr(text,"decel=0.60") && strstr(text,"wacc=3.0") &&
           strstr(text,"turn=0.120"));
    assert(mission_format_params(&m,text,8)<8);                                 /* truncates safely */
    assert(mission_set_param(&m,"EDGE",1.01f) && m.cfg.edge_m==1.01f);        /* nominal edge */
    assert(!mission_set_param(&m,"edge",0.4f) && m.cfg.edge_m==1.01f);
    assert(mission_format_params(&m,text,sizeof(text))>0 && strstr(text,"edge=1.01"));
    assert(mission_set_param(&m,"CIRCLE",1.0f) && m.cfg.circle==1.0f);           /* circle task */
    assert(!mission_set_param(&m,"circle",2.0f) && m.cfg.circle==1.0f);
    assert(mission_format_params(&m,text,sizeof(text))>0 && strstr(text,"circle=1"));
    assert(mission_start(&m,1));
    assert(!mission_set_param(&m,"kg",2.0f) && m.cfg.yaw_rate_kg==1.0f);        /* not while running */
}
static void test_straight_w_slew(void) {
    chassis_mission_t slow, fast; mission_input_t in={0};
    mission_config_t c=cfg; c.line_angular_acceleration=1.0f;
    mission_init(&slow,&c); mission_init(&fast,&cfg);
    assert(mission_start(&slow,1) && mission_start(&fast,1));
    in.dt=0.01f; in.line_valid=in.imu_valid=in.new_line=1; in.line_mask=0x03;   /* line far left */
    for (int i=0;i<3;i++) {
        in.left_delta_m=in.right_delta_m=0.002f;
        mission_step(&slow,&in); mission_step(&fast,&in);
    }
    assert(fabsf(slow.w)<=1.0f*0.03f+1e-6f);   /* 1 rad/s^2 for 30 ms */
    assert(fabsf(fast.w)>fabsf(slow.w)*2.0f);  /* 0 = the turn slew (3 rad/s^2 here) */
}
int main(void) {
    test_line_and_faults(); test_confirmation_and_preturn(); test_wheel_yaw_rate_loop();
    test_imu_dropout_tolerance(); test_lost_line_search_direction();
    test_turn_overshoot_with_lag(); test_reacquire_offset_line(); test_heading_hold();
    test_reacquire_waits_for_rotation_to_stop(); test_heading_reference_follows_a_centred_line();
    for (unsigned n=1;n<=5;n++) test_route(n);
    test_route_from(1,0.95f); test_slow_before_known_corner(); test_turn_rate_uses_measured_speed();
    test_braking_starts_at_the_braking_distance(); test_first_edge_is_capped();
    test_stall_and_turn_timeout(); test_set_turn_speed(); test_set_param_and_format();
    test_straight_w_slew(); test_run_progress(1,0.4f); test_run_progress(2,0.25f);
    cfg.turn_m_s=0.09f; test_route(3); cfg.turn_m_s=0.12f;   /* slower corners still complete */
    puts("Mission tests passed: steering, fresh scans, holds, 1..5 laps, finish, faults, stop.");
    return 0;
}
