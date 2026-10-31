"""Small planar geometry kernel, no NumPy/OpenCV dependency on the board.

Camera axes: x right, y down, z forward. Plane axes: x right, y down, mm.
Requires a non-mirrored landscape target with its physical top edge on top.
"""
import math
from protocol import finite


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def scale(a, k):
    return tuple(v * k for v in a)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    length = norm(a)
    if length < 1e-10:
        raise ValueError("degenerate vector")
    return scale(a, 1.0 / length)


def solve(matrix, rhs):
    n = len(rhs)
    a = [list(matrix[i]) + [rhs[i]] for i in range(n)]
    for c in range(n):
        pivot = max(range(c, n), key=lambda r: abs(a[r][c]))
        if abs(a[pivot][c]) < 1e-12:
            raise ValueError("singular quadrilateral")
        a[c], a[pivot] = a[pivot], a[c]
        divisor = a[c][c]
        a[c] = [v / divisor for v in a[c]]
        for r in range(n):
            if r != c:
                factor = a[r][c]
                a[r] = [a[r][j] - factor * a[c][j] for j in range(n + 1)]
    return tuple(a[i][n] for i in range(n))


def homography(source, destination):
    rows, rhs = [], []
    for (x, y), (u, v) in zip(source, destination):
        rows.append((x, y, 1, 0, 0, 0, -u*x, -u*y)); rhs.append(u)
        rows.append((0, 0, 0, x, y, 1, -v*x, -v*y)); rhs.append(v)
    return solve(rows, rhs) + (1.0,)


def transform(h, point):
    x, y = point
    denominator = h[6]*x + h[7]*y + h[8]
    if abs(denominator) < 1e-10:
        raise ValueError("point at infinity")
    return ((h[0]*x + h[1]*y + h[2]) / denominator,
            (h[3]*x + h[4]*y + h[5]) / denominator)


def polygon_area(points):
    return abs(sum(points[i][0]*points[(i+1) % len(points)][1] -
                   points[(i+1) % len(points)][0]*points[i][1] for i in range(len(points)))) * 0.5


def order_quad(points):
    """Convex TL,TR,BR,BL; intended for modest camera roll, not portrait targets."""
    if len(points) != 4 or len(set(tuple(p) for p in points)) != 4:
        raise ValueError("four distinct corners required")
    cx = sum(p[0] for p in points) / 4
    cy = sum(p[1] for p in points) / 4
    result = sorted([tuple(p) for p in points], key=lambda p: math.atan2(p[1]-cy, p[0]-cx))
    first = min(range(4), key=lambda i: result[i][0] + result[i][1])
    result = result[first:] + result[:first]
    signs = []
    for i in range(4):
        a, b, c = result[i], result[(i+1) % 4], result[(i+2) % 4]
        signs.append((b[0]-a[0])*(c[1]-b[1]) - (b[1]-a[1])*(c[0]-b[0]))
    if min(signs) <= 1e-5:
        raise ValueError("non-convex quadrilateral")
    return tuple(result)


class CameraModel:
    def __init__(self, width, height, fx, fy, cx, cy, distortion=()):
        self.width, self.height = width, height
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.distortion = tuple(distortion) if distortion else (0, 0, 0, 0, 0)
        if (len(self.distortion) != 5 or any(not finite(v) for v in
                (fx, fy, cx, cy) + self.distortion) or fx <= 0 or fy <= 0):
            raise ValueError("invalid camera calibration")

    def normalized(self, pixel):
        xd, yd = (pixel[0]-self.cx)/self.fx, (pixel[1]-self.cy)/self.fy
        x, y = xd, yd
        k1, k2, p1, p2, k3 = self.distortion
        for _ in range(12):
            r2 = x*x + y*y
            radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
            if abs(radial) < 1e-8:
                raise ValueError("invalid distortion correction")
            x, y = ((xd - 2*p1*x*y - p2*(r2+2*x*x)) / radial,
                    (yd - p1*(r2+2*y*y) - 2*p2*x*y) / radial)
        # A fixed iteration count is not proof of convergence, especially at edges.
        # Reject rather than silently using the wrong camera ray in laser calibration.
        if not finite(x) or not finite(y):
            raise ValueError("nonfinite distortion correction")
        recovered = self.project((x, y, 1.0))
        if (any(not finite(v) for v in recovered) or
                (recovered[0]-pixel[0])**2 + (recovered[1]-pixel[1])**2 > 0.25**2):
            raise ValueError("distortion correction did not converge")
        return (x, y)

    def project(self, point):
        if point[2] <= 0:
            raise ValueError("point behind camera")
        x, y = point[0]/point[2], point[1]/point[2]
        r2 = x*x + y*y
        k1, k2, p1, p2, k3 = self.distortion
        radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
        return (self.fx*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+self.cx,
                self.fy*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+self.cy)


class PlanePose:
    def __init__(self, r1, r2, translation):
        self.r1, self.r2, self.t = r1, r2, translation
        self.normal = unit(cross(r1, r2))

    def camera_point(self, plane_point):
        return add(self.t, add(scale(self.r1, plane_point[0]), scale(self.r2, plane_point[1])))

    def plane_point(self, camera_point):
        relative = sub(camera_point, self.t)
        return (dot(relative, self.r1), dot(relative, self.r2))

    def intersection(self, origin, direction):
        denominator = dot(self.normal, direction)
        if abs(denominator) < 1e-8:
            raise ValueError("ray parallel to target")
        distance = dot(self.normal, sub(self.t, origin)) / denominator
        if distance <= 0:
            raise ValueError("intersection behind ray")
        return add(origin, scale(direction, distance))


def rotate(axis_angle, v):
    """Rodrigues: v rotated by the axis-angle vector (rad)."""
    angle = norm(axis_angle)
    if angle < 1e-12:
        return add(v, cross(axis_angle, v))
    k = scale(axis_angle, 1.0 / angle)
    c, s = math.cos(angle), math.sin(angle)
    return add(add(scale(v, c), scale(cross(k, v), s)), scale(k, dot(k, v) * (1.0 - c)))


def reprojection_residuals(pose, plane, corners, camera):
    """(du, dv) per corner in pixels, flattened."""
    out = []
    for p, q in zip(plane, corners):
        u, v = camera.project(pose.camera_point(p))
        out.append(u - q[0])
        out.append(v - q[1])
    return out


def max_residual(residuals):
    return max(math.sqrt(residuals[i]**2 + residuals[i+1]**2) for i in range(0, len(residuals), 2))


def _updated(pose, delta):
    w, dt = delta[:3], delta[3:]
    return PlanePose(rotate(w, pose.r1), rotate(w, pose.r2), add(pose.t, dt))


def _normalized_residuals(pose, plane, normalized, fx, fy):
    """(fx*(X/Z - xn), fy*(Y/Z - yn)) per corner: pixel-scaled, distortion already removed."""
    out = []
    for (px, py), (xn, yn) in zip(plane, normalized):
        p = pose.camera_point((px, py))
        if p[2] <= 0:
            raise ValueError("point behind camera")
        out.append(fx * (p[0] / p[2] - xn))
        out.append(fy * (p[1] / p[2] - yn))
    return out


# Real frames (120 photos): residual median 0.82 -> 0.77 px and max 2.26 -> 1.53 px within
# 2 iterations, unchanged after. The first version (numeric Jacobian, distortion in every
# projection) took ~20 ms per frame on the K230: frames 40 -> 60 ms apart (field 2026-10-28).
POSE_ITERATIONS = 2


def refine_pose(pose, plane, normalized, fx, fy, iterations=POSE_ITERATIONS):
    """Gauss-Newton (with a Levenberg-Marquardt fallback) on rotation (axis-angle, applied on
    the left) and translation, minimising the pixel-scaled reprojection error of the corners
    in undistorted normalised coordinates, with the analytic Jacobian. The homography pose
    is only the start: its rigidification (r1 normalised, r2 orthogonalised) left 5+ px
    residuals on small, far targets (field 2026-10-28: 14-23 % of the far-side frames were
    rejected by the gate)."""
    try:
        residuals = _normalized_residuals(pose, plane, normalized, fx, fy)
    except ValueError:
        return pose
    cost = sum(r*r for r in residuals)
    for _ in range(iterations):
        jtj = [[0.0] * 6 for _ in range(6)]
        jtr = [0.0] * 6
        for i, (px, py) in enumerate(plane):
            q = add(scale(pose.r1, px), scale(pose.r2, py))       # R p
            x, y, z = add(q, pose.t)
            iz = 1.0 / z
            ax, az = iz, -x * iz * iz                                # d(X/Z)/dP = (ax, 0, az)
            by, bz = iz, -y * iz * iz                                # d(Y/Z)/dP = (0, by, bz)
            # dP/dw = -[q]x, dP/dt = I
            ju = (fx * az * q[1], fx * (ax * q[2] - az * q[0]), -fx * ax * q[1], fx * ax, 0.0, fx * az)
            jv = (fy * (bz * q[1] - by * q[2]), -fy * bz * q[0], fy * by * q[0], 0.0, fy * by, fy * bz)
            for row, r in ((ju, residuals[2*i]), (jv, residuals[2*i+1])):
                for a in range(6):
                    jtr[a] += row[a] * r
                    for b in range(a, 6):
                        jtj[a][b] += row[a] * row[b]
        for a in range(6):
            for b in range(a):
                jtj[a][b] = jtj[b][a]
        improved = False
        damping = 0.0
        for _attempt in range(4):
            system = [[jtj[a][b] + (damping * jtj[a][a] if a == b else 0.0) for b in range(6)]
                      for a in range(6)]
            try:
                candidate = _updated(pose, solve(system, [-v for v in jtr]))
                candidate_residuals = _normalized_residuals(candidate, plane, normalized, fx, fy)
            except ValueError:
                damping = damping * 10.0 if damping else 1e-3
                continue
            candidate_cost = sum(r*r for r in candidate_residuals)
            if candidate_cost < cost:
                converged = candidate_cost > cost * 0.999
                pose, residuals, cost = candidate, candidate_residuals, candidate_cost
                improved = True
                break
            damping = damping * 10.0 if damping else 1e-3
        if not improved or converged or cost < 1e-6:
            break
    return pose


def estimate_pose(corners, camera, width_mm=297.0, height_mm=210.0, max_error_px=3.0):
    plane = ((0, 0), (width_mm, 0), (width_mm, height_mm), (0, height_mm))
    normalized = [camera.normalized(p) for p in corners]
    h = homography(plane, normalized)
    c1, c2, c3 = (h[0],h[3],h[6]), (h[1],h[4],h[7]), (h[2],h[5],h[8])
    n1, n2 = norm(c1), norm(c2)
    if min(n1, n2) < 1e-10 or abs(n1-n2)/max(n1,n2) > 0.25:
        raise ValueError("inconsistent target dimensions or calibration")
    factor = 2.0/(n1+n2)
    if c3[2] < 0: factor = -factor
    r1 = unit(scale(c1, factor))
    raw2 = scale(c2, factor)
    r2 = unit(sub(raw2, scale(r1, dot(r1, raw2))))
    pose = refine_pose(PlanePose(r1, r2, scale(c3, factor)), plane, normalized, camera.fx, camera.fy)
    error = max_residual(reprojection_residuals(pose, plane, corners, camera))
    if error > max_error_px:
        # Value and gate in the reason: the K230 log then shows how far over the gate it is.
        raise ValueError("pose reprojection error {:.1f}>{:.1f}px".format(error, max_error_px))
    return pose, error


def aim_solution(pose, origin, direction, target_mm):
    direction = unit(direction)
    hit = pose.intersection(origin, direction)
    hit_mm = pose.plane_point(hit)
    target_cam = pose.camera_point(target_mm)
    desired = sub(target_cam, origin)
    if desired[2] <= 0 or direction[2] <= 0:
        raise ValueError("target/ray behind camera")
    yaw = math.atan2(desired[0], desired[2]) - math.atan2(direction[0], direction[2])
    pitch = math.atan2(desired[1], math.sqrt(desired[0]**2+desired[2]**2)) - \
            math.atan2(direction[1], math.sqrt(direction[0]**2+direction[2]**2))
    return {"yaw_deg": math.degrees(yaw), "pitch_deg": math.degrees(pitch),
            "hit_mm": hit_mm, "target_mm": target_mm,
            "error_mm": (target_mm[0]-hit_mm[0], target_mm[1]-hit_mm[1]),
            "depth_mm": target_cam[2], "range_mm": norm(target_cam),
            "plane_distance_mm": abs(dot(pose.normal, pose.t))}


class TargetPoint:
    """Static center now. Future circle controller can change the desired plane point."""
    def __init__(self, center=(148.5,105.0), width=297.0, height=210.0):
        self.point = center
        self.width, self.height = width, height

    def set_point(self, x_mm, y_mm):
        if not 0 <= x_mm <= self.width or not 0 <= y_mm <= self.height:
            raise ValueError("target point outside paper")
        self.point = (x_mm, y_mm)

    def current(self):
        return self.point


def circle_point(center, radius_mm, progress_laps, direction=1):
    """Plane point (mm, x right, y down) on the circle task for the run progress in laps: starts
    at the top; direction 1 = counter-clockwise as seen facing the target, -1 = clockwise."""
    angle = 2.0 * math.pi * progress_laps
    return (center[0] - direction * radius_mm * math.sin(angle),
            center[1] - radius_mm * math.cos(angle))
