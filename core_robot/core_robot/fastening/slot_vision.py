"""손목 카메라 영상에서 십자 홈 각도를 잰다.

전제: 카메라가 툴(J6)과 함께 돈다. 그래서 화면 속 홈 각도 = 일자 날과 홈의 상대 각도 + 상수.
    - 그 상수(날이 홈에 맞았을 때의 화면 각도)를 보정 때 ref_deg 로 저장한다.
    - 툴이 볼트 바로 위(같은 상대 위치)에 있으면 볼트는 J6 를 돌려도 화면의 같은 픽셀에 있다
      → ROI(볼트 머리 원)도 보정 때 한 번 저장하면 된다.
    - 드라이버를 다시 쥐면 날 방향이 바뀌므로 보정을 다시 한다.
십자는 90° 마다 같은 모양이므로 각도는 [-45, 45) 로 다룬다.
"""
import json
import math
import os

import cv2
import numpy as np

CALIB_PATH = os.path.expanduser("~/cobot2_ws/logs/vision_calib.json")

METHOD = "blob"         # "blob"(기본, 11/26 16:31 원본으로 선택): 머리 안의 어두운 십자 구멍을 잘라
                        #   중심에서 뻗은 방향을 90° 주기로 평균 — 장난감 볼트의 짧고 굵은 십자에 맞는 방식
                        # "hough": 직선 검출 → 90° 주기 평균 (가늘고 긴 홈용)
                        # "gradient": ROI 모든 픽셀의 기울기 방향 평균 — 합성 시험에서 ROI 에 드라이버 끝이
                        #   걸리면 더 크게 끌려가서(최대 오차 6°) 기본으로 쓰지 않는다. 실측 원본으로 다시 비교할 것.
                        # 어느 방식이든 ROI 안에 카메라와 같이 도는 물체(드라이버 끝 · 그리퍼)가 들어가면
                        #   툴을 돌려도 각도가 덜 바뀐다 → 11/26 16:14 보정 실패(20° 회전에 7.3° 변화)의 유력한 원인.
INNER_RATIO = 0.75      # 머리 테두리(원) 가장자리는 빼고 안쪽만 본다
GRAD_SIZE = 128         # gradient: ROI 를 이 크기로 늘려서 계산 (머리 크기와 상관없이 같은 기준)
GRAD_MIN_CONF = 0.12    # gradient 한 장 판정 기준 (합성 영상으로 정함 — 실측 후 조정)
GRAD_MULTI_MIN_CONF = 0.10
CANNY_LO, CANNY_HI = 40, 120
HOUGH_THRESH = 12
MIN_LINE_RATIO = 0.35   # 선 길이 ≥ r * 이 비율
MIN_CONF = 0.55         # 한 장 판정 결집도(0~1). 낮으면 판정 불가
MULTI_MIN_CONF = 0.45   # 여러 장 합친 판정 기준 (합치면 노이즈 선이 상쇄돼 결집도가 안정)
MIN_TOTAL_LEN_RATIO = 1.0


def wrap90(a):
    """각도를 [-45, 45) 로."""
    return (a + 45.0) % 90.0 - 45.0


def _crop(img, roi):
    cx, cy, r = [int(v) for v in roi]
    h, w = img.shape[:2]
    x0, y0, x1, y1 = max(0, cx - r), max(0, cy - r), min(w, cx + r), min(h, cy + r)
    return img[y0:y1, x0:x1], (cx - x0, cy - y0), x0, y0


def _accumulate_gradient(img, roi):
    """ROI 안 밝기 기울기 방향(θ)을 크기로 가중해 4θ 벡터로 합친다.
    십자 홈의 네 변은 기울기 방향이 90° 주기로 모이고, 원 테두리 · 노이즈는 방향이 고르게 퍼져 상쇄된다."""
    if img is None:
        return 0.0, 0.0, 0.0, []
    crop, (ccx, ccy), _, _ = _crop(img, roi)
    if crop.size == 0:
        return 0.0, 0.0, 0.0, []
    r = int(roi[2])
    k = GRAD_SIZE / max(1, 2 * r)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4)).apply(gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 0).astype(np.float32)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = cv2.magnitude(gx, gy)
    th = np.arctan2(gy, gx)
    mask = np.zeros(gray.shape, np.uint8)
    cv2.circle(mask, (int(ccx * k), int(ccy * k)), int(r * k * INNER_RATIO), 1, -1)
    w = mag * mask
    c = float(np.sum(w * np.cos(4 * th)))
    s_ = float(np.sum(w * np.sin(4 * th)))
    return c, s_, float(np.sum(w)), []


BLOB_INNER = 0.85       # blob: 머리 원의 이 비율 안에서만 어두운 픽셀을 찾는다
BLOB_DARK_FRAC = 0.25   # blob: 원 안에서 가장 어두운 이 비율을 십자 구멍으로 본다
BLOB_MIN_CONF = 0.10    # blob: conf 는 십자 유무를 못 가린다(책상 0.22, 볼트2 0.14) — 유효성은 보정의 ±20° 회전 시험으로 본다
BLOB_MULTI_MIN_CONF = 0.10


def _blob_pixels(img, roi):
    """머리 원 안에서 가장 어두운 십자 덩어리의 픽셀. 반환 (xs, ys, k, x0, y0) — xs·ys 는 k 배 늘린 crop 좌표."""
    if img is None:
        return None
    crop, (ccx, ccy), x0, y0 = _crop(img, roi)
    if crop.size == 0:
        return None
    r = int(roi[2])
    k = GRAD_SIZE / max(1, 2 * r)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    mask = np.zeros(gray.shape, np.uint8)
    rc = int(r * k * BLOB_INNER)
    cv2.circle(mask, (int(ccx * k), int(ccy * k)), rc, 255, -1)
    vals = gray[mask > 0]
    if vals.size < 50:
        return None
    # 가장 어두운 BLOB_DARK_FRAC 만 십자로 본다 — ROI 에 머리 밖 배경이 조금 섞여도 십자 구멍이 제일 어둡다
    thr = np.percentile(vals, BLOB_DARK_FRAC * 100.0)
    dark = ((gray <= thr) & (mask > 0)).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stats, cents = cv2.connectedComponentsWithStats(dark)
    if n <= 1:
        return None
    # 머리 중심에 가장 가까운 큰 덩어리
    centre = np.array([ccx * k, ccy * k])
    best = max(range(1, n), key=lambda i: stats[i, cv2.CC_STAT_AREA] / (1.0 + np.linalg.norm(cents[i] - centre)))
    ys, xs = np.nonzero(lab == best)
    return xs, ys, k, x0, y0


def _accumulate_blob(img, roi):
    """십자 덩어리 픽셀 방향(중심 기준 φ)을 거리² 가중으로 4φ 벡터 합.
    팔 끝(중심에서 먼 픽셀)이 팔 방향을 강하게 대표한다. 원형 덩어리면 상쇄돼 conf ≈ 0."""
    blob = _blob_pixels(img, roi)
    if blob is None:
        return 0.0, 0.0, 0.0, []
    xs, ys = blob[0], blob[1]
    dx, dy = xs - xs.mean(), ys - ys.mean()
    w = dx * dx + dy * dy
    phi = np.arctan2(dy, dx)
    c = float(np.sum(w * np.cos(4 * phi)))
    s_ = float(np.sum(w * np.sin(4 * phi)))
    return c, s_, float(np.sum(w)), []


def cross_center(frames, roi):
    """십자 구멍 중심(= 나사 축)의 원본 픽셀 좌표. 여러 장의 중앙값. 반환 (u, v) 또는 None."""
    pts = []
    for f in frames:
        blob = _blob_pixels(f, roi)
        if blob is None:
            continue
        xs, ys, k, x0, y0 = blob
        pts.append((x0 + xs.mean() / k, y0 + ys.mean() / k))
    if len(pts) < max(1, len(frames) // 2):
        return None
    arr = np.array(pts)
    return float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))


def _accumulate(img, roi):
    if METHOD == "blob":
        return _accumulate_blob(img, roi)
    if METHOD == "gradient":
        return _accumulate_gradient(img, roi)
    return _accumulate_hough(img, roi)


def _accumulate_hough(img, roi):
    """ROI 안 선 성분을 90° 주기 벡터로 합친다. 반환 (c, s, total, lines)."""
    if img is None:
        return 0.0, 0.0, 0.0, []
    cx, cy, r = [int(v) for v in roi]
    h, w = img.shape[:2]
    x0, y0, x1, y1 = max(0, cx - r), max(0, cy - r), min(w, cx + r), min(h, cy + r)
    crop = img[y0:y1, x0:x1]
    if crop.size == 0:
        return 0.0, 0.0, 0.0, []
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4)).apply(gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, CANNY_LO, CANNY_HI)
    mask = np.zeros_like(edges)
    cv2.circle(mask, (cx - x0, cy - y0), int(r * INNER_RATIO), 255, -1)
    edges &= mask
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, HOUGH_THRESH,
                            minLineLength=max(5, int(r * MIN_LINE_RATIO)), maxLineGap=4)
    if lines is None:
        return 0.0, 0.0, 0.0, []
    s = c = total = 0.0
    out = []
    for x_a, y_a, x_b, y_b in lines[:, 0]:
        dx, dy = x_b - x_a, y_b - y_a
        length = math.hypot(dx, dy)
        th = math.atan2(dy, dx)
        c += length * math.cos(4 * th)
        s += length * math.sin(4 * th)
        total += length
        out.append((x_a + x0, y_a + y0, x_b + x0, y_b + y0))
    return c, s, total, out


def _min_conf(multi):
    if METHOD == "blob":
        return BLOB_MULTI_MIN_CONF if multi else BLOB_MIN_CONF
    if METHOD == "gradient":
        return GRAD_MULTI_MIN_CONF if multi else GRAD_MIN_CONF
    return MULTI_MIN_CONF if multi else MIN_CONF


def _judge(c, s, total, r, n_frames, min_conf):
    if total <= 0:
        return None, 0.0
    if METHOD == "hough" and total < r * MIN_TOTAL_LEN_RATIO * n_frames * 0.5:
        return None, 0.0
    conf = math.hypot(c, s) / total
    if conf < min_conf:
        return None, conf
    return wrap90(math.degrees(math.atan2(s, c) / 4.0)), conf


def slot_angle(img, roi):
    """한 장으로 판정. roi = (cx, cy, r) 픽셀. 반환 (angle_deg | None, conf, lines)."""
    c, s, total, lines = _accumulate(img, roi)
    angle, conf = _judge(c, s, total, roi[2], 1, _min_conf(multi=False))
    return angle, conf, lines


def slot_angle_multi(frames, roi, min_conf=None):
    """여러 장의 선 성분을 합쳐 판정 (프레임 노이즈 상쇄). 반환 (angle | None, conf, 마지막 장 lines, 장수)."""
    frames = [f for f in frames if f is not None]
    if not frames:
        return None, 0.0, [], 0
    C = S = T = 0.0
    lines = []
    for f in frames:
        c, s, total, lines = _accumulate(f, roi)
        C, S, T = C + c, S + s, T + total
    angle, conf = _judge(C, S, T, roi[2], len(frames), _min_conf(multi=True) if min_conf is None else min_conf)
    return angle, conf, lines, len(frames)


def to90(a):
    """표시용 [0, 90) — 홈이 45° 근처일 때 ±45 로 튀어 보이지 않게."""
    return None if a is None else a % 90.0


def draw(img, roi, angle, conf, lines, ref=None):
    vis = img.copy()
    cx, cy, r = [int(v) for v in roi]
    cv2.circle(vis, (cx, cy), r, (0, 255, 255), 1)
    cv2.circle(vis, (cx, cy), int(r * INNER_RATIO), (0, 200, 200), 1)
    for x_a, y_a, x_b, y_b in lines:
        cv2.line(vis, (int(x_a), int(y_a)), (int(x_b), int(y_b)), (255, 0, 255), 1)

    def cross(a, color):
        for k in (0, 90):
            t = math.radians(a + k)
            dx, dy = r * math.cos(t), r * math.sin(t)
            cv2.line(vis, (int(cx - dx), int(cy - dy)), (int(cx + dx), int(cy + dy)), color, 2)

    if ref is not None:
        cross(ref, (0, 255, 0))
    if angle is not None:
        cross(angle, (0, 0, 255))
    txt = f"angle={to90(angle):.1f} conf={conf:.2f}" if angle is not None else f"angle=None conf={conf:.2f}"
    if ref is not None:
        txt += f" ref={to90(ref):.1f}"
        if angle is not None:
            txt += f" err={wrap90(ref - angle):+.1f}"
    cv2.putText(vis, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return vis


def load_calib(path=CALIB_PATH):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def save_calib(data, path=CALIB_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


ZOOM = 5                # ROI 고를 때 확대 배율
ZOOM_HALF = 70          # 확대할 영역 반폭(px, 원본 기준)


def pick_roi(img, window="pick bolt head"):
    """1) 전체 화면에서 볼트 근처를 한 번 클릭 → 2) 5배 확대 화면에서 머리 중심 → 테두리 클릭 → Enter.
    r = 처음부터 다시, q = 취소. 원은 머리 테두리에 딱 맞게, 드라이버 끝이 원 안에 들어가지 않게."""
    state = {"stage": 0, "anchor": None, "pts": []}

    def on_mouse(ev, x, y, *_):
        if ev != cv2.EVENT_LBUTTONDOWN:
            return
        if state["stage"] == 0:
            state["anchor"] = (x, y)
            state["stage"] = 1
        elif len(state["pts"]) < 2:
            state["pts"].append((x, y))

    h, w = img.shape[:2]
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    while True:
        if state["stage"] == 0:
            vis = img.copy()
            cv2.putText(vis, "1) click near the bolt head | q=quit", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            ax, ay = state["anchor"]
            x0, y0 = min(max(0, ax - ZOOM_HALF), w - 2 * ZOOM_HALF), min(max(0, ay - ZOOM_HALF), h - 2 * ZOOM_HALF)
            crop = img[y0:y0 + 2 * ZOOM_HALF, x0:x0 + 2 * ZOOM_HALF]
            vis = cv2.resize(crop, None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_CUBIC)
            cv2.putText(vis, "2) click head CENTER -> head EDGE | Enter=ok r=reset q=quit", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            pts = state["pts"]
            for p in pts:
                cv2.circle(vis, p, 4, (0, 0, 255), -1)
            if len(pts) == 2:
                rr = int(math.dist(pts[0], pts[1]))
                cv2.circle(vis, pts[0], rr, (0, 255, 255), 2)
                cv2.circle(vis, pts[0], int(rr * INNER_RATIO), (0, 200, 200), 1)
        cv2.imshow(window, vis)
        k = cv2.waitKey(30) & 0xFF
        if k == ord("r"):
            state.update(stage=0, anchor=None, pts=[])
        elif k == ord("q"):
            cv2.destroyWindow(window)
            return None
        elif k in (13, 10) and state["stage"] == 1 and len(state["pts"]) == 2:
            cv2.destroyWindow(window)
            (px, py), (ex, ey) = state["pts"]
            cx, cy = x0 + px / ZOOM, y0 + py / ZOOM
            r = math.dist((px, py), (ex, ey)) / ZOOM
            return [int(round(cx)), int(round(cy)), max(4, int(round(r)))]
