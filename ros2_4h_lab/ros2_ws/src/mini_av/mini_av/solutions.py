"""정답 예시. 먼저 my_algorithms.py 를 직접 채워보고, 막히면 여기를 본다.

노드는 파라미터 use_solution:=true 이면 이 파일의 함수를, 아니면 my_algorithms.py 를 쓴다.
"""
import math

import numpy as np


# ---------------------------------------------------------------------------
# Lab 3-1. 차선 인식: BEV 영상 → 차선 중심의 좌우 오프셋 [m]
# ---------------------------------------------------------------------------
def find_lane_center(img, cam, lane_width, state):
    """
    img   : (H, W) uint8 흑백 BEV 영상. 검은 테이프 = 어두운 픽셀
    cam   : BevCamera (cam.col_to_y(col) 로 열 → 좌우 거리[m] 변환)
    state : dict. 프레임 사이에 값을 기억할 때 쓴다 (직전 차선 중심 등)
    반환  : 차선 중심의 좌우 오프셋 [m] (왼쪽 +). 차선을 못 찾으면 float('nan')
    """
    H, W = img.shape
    # 1) ROI: 전방 0.25~0.45 m 정도 띠만 본다 (너무 멀면 곡선에서 선이 화면 밖으로 나감)
    roi = img[int(H * 0.40):int(H * 0.80), :]

    # 2) 이진화: 조명이 고르지 않으니 고정값 대신 ROI 밝기 기준으로 threshold
    thresh = min(110, float(np.percentile(roi, 50)) * 0.55)
    mask = roi < thresh

    # 3) 열별 히스토그램
    hist = mask.sum(axis=0).astype(float)
    min_count = 0.25 * roi.shape[0]

    # 4) 직전 차선 중심 열을 기준으로 왼쪽/오른쪽 선을 나눈다 (화면 중앙 고정보다 곡선에 강함)
    split = int(state.get("split_col", W // 2))
    cols = np.arange(W)
    left_y = right_y = None
    lh, rh = hist[:split], hist[split:]
    if lh.max(initial=0) >= min_count:
        sel = lh >= min_count
        left_y = cam.col_to_y(float((cols[:split][sel] * lh[sel]).sum() / lh[sel].sum()))
    if rh.max(initial=0) >= min_count:
        sel = rh >= min_count
        right_y = cam.col_to_y(float((cols[split:][sel] * rh[sel]).sum() / rh[sel].sum()))

    # 5) 차선 중심 계산 + 한쪽 선만 보일 때 대응
    if left_y is not None and right_y is not None:
        center = (left_y + right_y) / 2
    elif left_y is not None:
        center = left_y - lane_width / 2
    elif right_y is not None:
        center = right_y + lane_width / 2
    else:
        return float("nan")

    # 다음 프레임의 분할 기준 갱신
    center_col = (cam.half_width - center) / (2 * cam.half_width) * (W - 1)
    state["split_col"] = int(np.clip(center_col, 10, W - 10))
    return float(center)


# ---------------------------------------------------------------------------
# Lab 3-2. 조향 제어: PD 제어
# ---------------------------------------------------------------------------
def steering_control(error, dt, state, kp, kd, max_w):
    """
    error : 차선 중심 오프셋 [m] (왼쪽 +). 왼쪽에 차선 중심이 있으면 왼쪽으로(+) 돌아야 한다.
    반환  : angular.z [rad/s]
    """
    prev = state.get("prev_error")
    d_err = 0.0 if (prev is None or dt <= 0) else (error - prev) / dt
    state["prev_error"] = error
    w = kp * error + kd * d_err
    return float(np.clip(w, -max_w, max_w))


# ---------------------------------------------------------------------------
# Lab 4-1. LiDAR: 전방 최소 거리
# ---------------------------------------------------------------------------
def front_distance(ranges, angle_min, angle_increment, half_fov_deg=20.0):
    """LaserScan 의 ranges 에서 전방 ±half_fov_deg 안의 최소 거리. 없으면 inf"""
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(r))
    angles = np.arctan2(np.sin(angles), np.cos(angles))   # -pi~pi 로 정규화
    sel = (np.abs(angles) <= math.radians(half_fov_deg)) & np.isfinite(r)
    return float(r[sel].min()) if sel.any() else float("inf")


# ---------------------------------------------------------------------------
# Lab 4-2. 판단: 유한 상태 기계 (FSM)
# ---------------------------------------------------------------------------
def decide(state, lane_error, obstacle_dist, base_speed, lost_time, dt):
    """
    반환: (새 상태 이름, 속도 배율 0~1)
      FOLLOW : 정상 주행
      SLOW   : 장애물이 1.0 m 안 → 감속
      STOP   : 장애물이 0.45 m 안 → 정지. 0.6 m 넘게 멀어지면 다시 출발 (히스테리시스)
      LOST   : 차선을 놓침 → 천천히 직전 조향 유지, 1.5초 넘으면 정지
    """
    lane_ok = not math.isnan(lane_error)

    if state == "STOP":
        if obstacle_dist > 0.6:
            return "FOLLOW", 1.0
        return "STOP", 0.0
    if obstacle_dist < 0.45:
        return "STOP", 0.0
    if not lane_ok:
        return "LOST", (0.4 if lost_time < 1.5 else 0.0)
    if obstacle_dist < 1.0:
        return "SLOW", 0.5
    return "FOLLOW", 1.0
