"""Kinematic model of one left corner: detection -> PRE_TURN -> TURN, with the body
speed and yaw rate lagging the commands. Reports where the axle lands relative to the
new line once the body has turned 90 deg (+ = outside the corner).
Run: python chassis/Tests/corner_sim.py
"""
import math

DT = 0.01
AHEAD = 0.095
DETECT_Y = -0.004          # probe row position when the corner is confirmed (3 scans)


def corner(v_cruise=0.20, v_turn=0.12, radius=0.085, alpha=8.0, decel=0.8,
           body_lag=0.2, lag_comp=0.20, lead=0.0, need_slow=True, use_vmeas=False):
    """lead: extra metres by which PRE_TURN ends early to cover the body lag."""
    x, y, th = 0.0, DETECT_Y - AHEAD, 0.0   # axle; old line along +y, new line is y=0 going -x
    v_cmd = v_body = v_cruise
    w_cmd = w_body = 0.0
    state, s_dist, s_time, ang = "PRE", 0.0, 0.0, 0.0
    for _ in range(600):
        if state == "PRE":
            tv, tw = v_turn, 0.0
            remaining = AHEAD - radius - lead - s_dist
            slow_ok = (v_cmd <= v_turn + 0.01) if need_slow else True
            if remaining <= 0 and s_time >= 0.06 and slow_ok:
                state, ang = "TURN", 0.0
        if state == "TURN":
            tv = v_turn
            rem = max(0.0, math.pi / 2 - ang - w_body * lag_comp)
            v_ref = v_body if use_vmeas else v_cmd
            tw = min(v_ref / radius, math.sqrt(2 * alpha * rem))
            if ang >= math.pi / 2 and abs(w_body) < 0.02:
                break
        v_cmd += max(-decel * DT, min(0.5 * DT, tv - v_cmd))
        w_cmd += max(-alpha * DT, min(alpha * DT, tw - w_cmd))
        v_body += (v_cmd - v_body) * DT / body_lag
        w_body += (w_cmd - w_body) * DT / body_lag
        x += -v_body * math.sin(th) * DT
        y += v_body * math.cos(th) * DT
        th += w_body * DT
        if state == "TURN":
            ang += w_body * DT
        s_dist += v_body * DT
        s_time += DT
    return y * 1000, math.degrees(th)


def main():
    print("old settings (0.12 m/s, R 0.085, no lead):  %+6.1f mm" % corner()[0])
    for lag in (0.1, 0.2, 0.3):
        common = dict(v_turn=0.10, radius=0.062, body_lag=lag, need_slow=False, use_vmeas=True,
                      lead=0.10 * (0.20 + (0.10 / 0.062) / (2 * 8)))
        slow = corner(v_cruise=0.10, **common)[0]
        fast = corner(v_cruise=0.20, **common)[0]
        print(f"firmware now, body lag {lag}s: slowed corner {slow:+5.1f} mm, first corner (full speed) {fast:+5.1f} mm")


if __name__ == "__main__":
    main()
