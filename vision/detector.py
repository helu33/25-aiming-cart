"""Black tape frame detector using the Yahboom cv2 subset (desktop testable)."""
import math
import cv2
from geometry import homography, transform, order_quad, polygon_area


def contour_points(contour):
    # Yahboom: (N,2); desktop: (N,1,2). Do not require np.float32 on ulab.
    result = []
    for row in contour:
        if len(row) == 1:
            row = row[0]
        if len(row) != 2:
            raise ValueError("unsupported contour shape")
        result.append((float(row[0]), float(row[1])))
    return result


def fit_line(points):
    """Total-least-squares line: (centroid, unit direction), or None if degenerate."""
    n = len(points)
    if n < 3:
        return None
    mx = sum(p[0] for p in points)/n
    my = sum(p[1] for p in points)/n
    sxx = sum((p[0]-mx)**2 for p in points)
    syy = sum((p[1]-my)**2 for p in points)
    sxy = sum((p[0]-mx)*(p[1]-my) for p in points)
    if sxx + syy < 1e-9:
        return None
    angle = 0.5*math.atan2(2*sxy, sxx-syy)
    return (mx, my), (math.cos(angle), math.sin(angle))


def intersect_lines(a, b):
    (px, py), (dx, dy) = a
    (qx, qy), (ex, ey) = b
    denominator = dx*ey - dy*ex
    if abs(denominator) < 1e-6:
        return None
    s = ((qx-px)*ey - (qy-py)*ex)/denominator
    return (px + s*dx, py + s*dy)


def refine_quad(quad, points, max_shift):
    """Sub-pixel corners from contour points near each corner.

    Each corner joins two lines fitted to the half of each adjacent side nearest
    that corner, so lens barrel curvature far from the corner has little effect.
    Corners that cannot be fitted or move more than max_shift keep their
    approxPolyDP position.
    """
    sides = []
    for k in range(4):
        a, b = quad[k], quad[(k+1) % 4]
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.sqrt(dx*dx + dy*dy)
        ux, uy = dx/length, dy/length
        tolerance = max(1.5, 0.04*length)
        start, end = [], []
        for x, y in points:
            rx, ry = x-a[0], y-a[1]
            t = (rx*ux + ry*uy)/length
            if abs(rx*uy - ry*ux) > tolerance:
                continue
            if 0.03 <= t <= 0.45:
                start.append((x, y))
            elif 0.55 <= t <= 0.97:
                end.append((x, y))
        chord = (a, (ux, uy))
        sides.append((fit_line(start) or chord, fit_line(end) or chord))
    refined = []
    for k in range(4):
        corner = intersect_lines(sides[k-1][1], sides[k][0])
        if corner is None or max(abs(corner[0]-quad[k][0]), abs(corner[1]-quad[k][1])) > max_shift:
            corner = quad[k]
        refined.append(corner)
    try:
        ordered = order_quad(refined)
    except ValueError:
        return quad
    return ordered if ordered == tuple(refined) else quad


def hierarchy_rows(hierarchy):
    if hierarchy is None:
        return []
    shape = getattr(hierarchy, "shape", ())
    if len(shape) == 3 or (not shape and len(hierarchy) == 1 and
                          len(hierarchy[0]) and hasattr(hierarchy[0][0], "__len__")):
        hierarchy = hierarchy[0]
    return [tuple(int(v) for v in row) for row in hierarchy]


class FrameDetector:
    def __init__(self, width, height, options, paper=(297.0,210.0), tape=18.0):
        self.width, self.height, self.options = width, height, options
        self.paper, self.tape = paper, tape
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                               (options["close_size"], options["close_size"]))
        self.last_center = None
        self.last_quad = None
        self.last_area = None
        self.confirmed = 0
        self.misses = 0
        self.last_status = "no_candidates"
        # For framelog.py: rectangles found before association, and the chosen (or largest)
        # one's area as a fraction of the image.
        self.last_candidates = 0
        self.last_area_fraction = 0.0
        # Binarisation threshold actually in use; starts at the configured value and
        # follows the scene when the tape is not found (lighting changes).
        self.threshold = float(options["threshold"])
        # Canny edges help close broken tape outlines but flood a busy, far scene with
        # contours (past max_contours); the search may switch them off and keep that.
        self.use_edges = bool(options["use_edges"])
        self._probe = 0

    def reset(self):
        self.last_center = None
        self.last_quad = self.last_area = None
        self.confirmed = self.misses = 0

    def _quad(self, contour):
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, self.options["approx_fraction"]*perimeter, True)
        points = contour_points(approx)
        if len(points) != 4:
            return None
        try:
            quad = order_quad(points)
        except ValueError:
            return None
        margin = self.options["border_margin_pixels"]
        for i, (x,y) in enumerate(quad):
            if not margin <= x < self.width-margin or not margin <= y < self.height-margin:
                return None
            other = quad[(i+1)%4]
            if math.sqrt((x-other[0])**2+(y-other[1])**2) < self.options["min_edge_pixels"]:
                return None
        return quad

    def _valid_inner(self, outer, inner):
        w,h = self.paper
        expected = ((self.tape,self.tape),(w-self.tape,self.tape),
                    (w-self.tape,h-self.tape),(self.tape,h-self.tape))
        try:
            mapping = homography(outer, ((0,0),(w,0),(w,h),(0,h)))
            actual = [transform(mapping, p) for p in inner]
        except ValueError:
            return False
        tolerance = self.options["inner_inset_tolerance_mm"]
        # Distinct inner corners must match the expected four tape inset corners.
        remaining = list(actual)
        for point in expected:
            nearest = min(range(len(remaining)), key=lambda i:
                          (remaining[i][0]-point[0])**2+(remaining[i][1]-point[1])**2)
            chosen = remaining.pop(nearest)
            if max(abs(chosen[0]-point[0]), abs(chosen[1]-point[1])) > tolerance:
                return False
        return True

    def _scene_mean(self, gray):
        # 16x12 sample grid: indexing works on numpy and on the board's ulab array.
        total = 0
        for i in range(12):
            y = (2*i + 1)*self.height//24
            for j in range(16):
                total += int(gray[y, (2*j + 1)*self.width//32])
        return total/192.0

    def _search_threshold(self, gray):
        """Try one alternative (threshold, edges) setting per frame (bounded cost) and
        adopt it if the frame is found. Thresholds: configured value, then fractions of
        the scene mean; each with and without Canny edges (if edges are enabled)."""
        ratios = self.options.get("threshold_ratios", ())
        thresholds = [float(self.options["threshold"])] + [r*self._scene_mean(gray) for r in ratios]
        edge_modes = (True, False) if self.options["use_edges"] else (False,)
        probes = [(t, e) for t in thresholds for e in edge_modes]
        # Closest to the setting in use first: field logs showed the frame dropping out for
        # 100-400 ms at the edge of a threshold (thr 70 <-> 83), also while standing still;
        # the nearest alternative usually finds it again on the next frame.
        probes.sort(key=lambda p: (p[1] != self.use_edges, abs(p[0] - self.threshold)))
        for _ in range(len(probes)):
            threshold, edges = probes[self._probe % len(probes)]
            self._probe += 1
            if abs(threshold - self.threshold) >= 3.0 or edges != self.use_edges:
                found = self.candidates(gray, threshold, edges)
                if found:
                    self.threshold, self.use_edges = threshold, edges
                return found
        return []

    def candidates(self, gray, threshold=None, use_edges=None):
        if threshold is None:
            threshold = self.threshold
        if use_edges is None:
            use_edges = self.use_edges
        shape = tuple(gray.shape)
        if shape != (self.height,self.width):
            raise ValueError("grayscale image shape mismatch: " + str(shape))
        size = self.options["blur_size"]
        blur = cv2.GaussianBlur(gray, (size,size), 0)
        _, binary = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY_INV)
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel)
        if use_edges:
            edges = cv2.Canny(blur, self.options["canny_low"], self.options["canny_high"])
            closed = cv2.bitwise_or(closed, edges)
        contours, hierarchy = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > self.options["max_contours"]:
            return []  # Reject clutter, do not spend unbounded time enumerating contours.
        rows = hierarchy_rows(hierarchy)
        if len(rows) != len(contours):
            raise ValueError("cv2 hierarchy unavailable or unsupported")
        areas = [abs(float(cv2.contourArea(c))) for c in contours]
        image_area = self.width*self.height
        results = []
        for i, contour in enumerate(contours):
            area = areas[i]
            if not self.options["min_area_fraction"] <= area/image_area <= self.options["max_area_fraction"]:
                continue
            outer = self._quad(contour)
            if outer is None:
                continue
            # Descendant hole contours up to 7 generations deep; edges can add
            # intervening contours between the tape's outer edge and its hole.
            stack = [(rows[i][2], 1)]
            while stack:
                j, generation = stack.pop()
                if j < 0:
                    continue
                stack.append((rows[j][0], generation))
                if generation < 7:
                    stack.append((rows[j][2], generation + 1))
                ratio = areas[j]/area
                if not self.options["min_inner_area_ratio"] <= ratio <= self.options["max_inner_area_ratio"]:
                    continue
                inner = self._quad(contours[j])
                if inner and self._valid_inner(outer, inner):
                    if self.options.get("refine_corners", True):
                        outer = refine_quad(outer, contour_points(contour),
                                            self.options.get("corner_refine_max_px", 3.0))
                    results.append({"corners": outer, "area": polygon_area(outer),
                                    "center": (sum(p[0] for p in outer)/4, sum(p[1] for p in outer)/4)})
                    break
        return results

    def detect(self, gray):
        candidates = self.candidates(gray)
        if candidates:
            self._probe = 0                  # next miss starts with the nearest probe again
        else:
            candidates = self._search_threshold(gray)
        had_candidates = bool(candidates)
        self.last_candidates = len(candidates)
        self.last_area_fraction = (max(c["area"] for c in candidates) / float(self.width * self.height)
                                   if candidates else 0.0)
        if self.last_center is not None:
            max_jump = self.options["max_jump_fraction"]*max(self.width,self.height)
            # Associate with the previous rectangle, not the largest new neighbor.
            # The image-size bound alone allowed a 128 px switch at 640x480.
            corner_limit = min(max_jump, max(10.0, math.sqrt(self.last_area) *
                               self.options.get("max_corner_shift_fraction", 0.35)))
            area_ratio = self.options.get("max_area_change_ratio", 1.8)
            nearby = []
            for candidate in candidates:
                if not 1.0/area_ratio <= candidate["area"]/self.last_area <= area_ratio:
                    continue
                error = sum((p[0]-q[0])**2+(p[1]-q[1])**2
                            for p,q in zip(candidate["corners"], self.last_quad)) / 4.0
                if error <= corner_limit**2:
                    candidate["association_error"] = error
                    nearby.append(candidate)
            candidates = nearby
        if not candidates:
            self.misses += 1
            # A brief dropout keeps the confirmation; only a longer loss re-confirms.
            if self.misses > self.options.get("miss_tolerance", 0):
                self.confirmed = 0
            if self.misses >= 10:
                self.reset()
            self.last_status = "association_rejected" if had_candidates else "no_candidates"
            return None
        chosen = (min(candidates, key=lambda c: c["association_error"])
                  if self.last_quad is not None else max(candidates, key=lambda c: c["area"]))
        self.last_center = chosen["center"]
        self.last_quad, self.last_area = chosen["corners"], chosen["area"]
        self.last_area_fraction = chosen["area"] / float(self.width * self.height)
        self.misses = 0
        self.confirmed = min(self.confirmed + 1, self.options["confirm_frames"])
        chosen["confirmed"] = self.confirmed >= self.options["confirm_frames"]
        self.last_status = "confirmed" if chosen["confirmed"] else "confirming"
        return chosen
