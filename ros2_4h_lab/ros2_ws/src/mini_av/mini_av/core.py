"""mini_av 시뮬레이터의 순수 파이썬 로직 (ROS 의존성 없음).

ROS 노드(sim_node.py)는 이 모듈을 감싸기만 한다.
그래서 ROS 없이도 tools/offline_check.py 로 동작을 검증할 수 있다.

좌표계 (ROS 표준, REP-103)
  - 월드(odom): x 오른쪽, y 위쪽, 단위 m
  - 차량(base_link): x 전방, y 왼쪽, 반시계 방향이 + 회전
"""
import math

import numpy as np


# ---------------------------------------------------------------------------
# 트랙: 경기장(stadium) 모양. 직선 2개 + 반원 2개. 흰 바닥에 검은 테이프 2줄.
# ---------------------------------------------------------------------------
class StadiumTrack:
    def __init__(self, straight=2.0, radius=0.8, lane_width=0.35, tape_width=0.03):
        self.L = straight          # 직선 구간 길이
        self.R = radius            # 반원 반지름 (차선 중심선 기준)
        self.lane_width = lane_width
        self.tape_width = tape_width

    def centerline_dist(self, x, y):
        """차선 중심선까지의 거리 (부호 없음). numpy 배열도 받는다."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        ax = np.abs(x)
        on_straight = ax <= self.L / 2
        d_straight = np.abs(np.abs(y) - self.R)
        d_curve = np.abs(np.hypot(ax - self.L / 2, y) - self.R)
        return np.where(on_straight, d_straight, d_curve)

    def is_tape(self, x, y):
        d = self.centerline_dist(x, y)
        return np.abs(d - self.lane_width / 2) < self.tape_width / 2

    def start_pose(self):
        # 아래쪽 직선 가운데에서 +x 방향 → 반시계 방향으로 돈다
        return 0.0, -self.R, 0.0


# ---------------------------------------------------------------------------
# 차량: 자전거(bicycle) 모델 + 액추에이터 지연. PiCar-X 같은 앞바퀴 조향 차량 흉내.
# 입력은 ROS 표준 Twist: linear.x = 속도[m/s], angular.z = 요레이트[rad/s].
# 조향각 한계 때문에 요레이트는 v * tan(max_steer) / wheelbase 로 잘린다.
# ---------------------------------------------------------------------------
class Car:
    def __init__(self, wheelbase=0.15, max_steer_deg=30.0, max_speed=0.8, tau=0.12):
        self.wheelbase = wheelbase
        self.max_steer = math.radians(max_steer_deg)
        self.max_speed = max_speed
        self.tau = tau             # 1차 지연 시정수 (모터·서보 반응 지연)
        self.reset(0.0, 0.0, 0.0)

    def reset(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = 0.0
        self.steer = 0.0

    def step(self, v_cmd, w_cmd, dt):
        v_cmd = float(np.clip(v_cmd, -self.max_speed, self.max_speed))
        # 요레이트 명령 → 조향각 명령
        if abs(v_cmd) > 1e-3:
            steer_cmd = math.atan(w_cmd * self.wheelbase / v_cmd)
        else:
            steer_cmd = 0.0
        steer_cmd = float(np.clip(steer_cmd, -self.max_steer, self.max_steer))

        a = dt / (self.tau + dt)
        self.v += a * (v_cmd - self.v)
        self.steer += a * (steer_cmd - self.steer)

        w = self.v * math.tan(self.steer) / self.wheelbase
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        return w


# ---------------------------------------------------------------------------
# 카메라: 이미 BEV(위에서 내려다본) 변환이 끝난 영상을 바로 만든다.
# 이미지 위쪽 = 먼 곳, 아래쪽 = 가까운 곳, 왼쪽 열 = 차량 왼쪽.
# ---------------------------------------------------------------------------
class BevCamera:
    def __init__(self, width=160, height=120, x_near=0.15, x_far=0.65, half_width=0.35):
        self.width, self.height = width, height
        self.x_near, self.x_far, self.half_width = x_near, x_far, half_width
        self._xs = np.linspace(x_far, x_near, height)[:, None]           # 행 → 전방 거리
        self._ys = np.linspace(half_width, -half_width, width)[None, :]  # 열 → 좌우 거리

    def col_to_y(self, col):
        """이미지 열 번호 → 차량 기준 좌우 거리 [m] (왼쪽 +)"""
        return self.half_width - col * (2 * self.half_width / (self.width - 1))

    def row_to_x(self, row):
        """이미지 행 번호 → 차량 기준 전방 거리 [m]"""
        return self.x_far - row * ((self.x_far - self.x_near) / (self.height - 1))

    def render(self, track, x, y, yaw, rng=None, noise=8.0, light_gradient=True):
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * self._xs - s * self._ys
        wy = y + s * self._xs + c * self._ys
        img = np.full((self.height, self.width), 225.0)
        if light_gradient:  # 조명 불균일 흉내 (왼쪽이 조금 어둡다)
            img -= np.linspace(40, 0, self.width)[None, :]
        img[track.is_tape(wx, wy)] = 35.0
        if rng is not None and noise > 0:
            img += rng.normal(0, noise, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# LiDAR: 360개 빔, 정사각형 박스에 대해 ray casting (slab method)
# 각도 0 = 차량 전방, 반시계 +. angle_min = -pi.
# ---------------------------------------------------------------------------
class Lidar:
    def __init__(self, n_beams=360, range_min=0.12, range_max=8.0, noise=0.005):
        self.n = n_beams
        self.range_min, self.range_max = range_min, range_max
        self.noise = noise
        self.angle_min = -math.pi
        self.angle_increment = 2 * math.pi / n_beams
        self.angles = self.angle_min + self.angle_increment * np.arange(n_beams)

    def scan(self, x, y, yaw, boxes, rng=None):
        """boxes: [(cx, cy, size), ...]  반환: ranges (np.ndarray, 못 맞추면 inf)"""
        ranges = np.full(self.n, np.inf)
        dx = np.cos(yaw + self.angles)
        dy = np.sin(yaw + self.angles)
        with np.errstate(divide="ignore", invalid="ignore"):
            for cx, cy, size in boxes:
                h = size / 2
                tx1, tx2 = (cx - h - x) / dx, (cx + h - x) / dx
                ty1, ty2 = (cy - h - y) / dy, (cy + h - y) / dy
                tnear = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
                tfar = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
                hit = (tfar >= tnear) & (tfar > 0)
                t = np.where(tnear > 0, tnear, tfar)
                ranges = np.where(hit & (t < ranges), t, ranges)
        if rng is not None and self.noise > 0:
            finite = np.isfinite(ranges)
            ranges[finite] += rng.normal(0, self.noise, finite.sum())
        ranges[(ranges < self.range_min) | (ranges > self.range_max)] = np.inf
        return ranges


def point_in_box(x, y, box, margin=0.0):
    cx, cy, size = box
    h = size / 2 + margin
    return abs(x - cx) <= h and abs(y - cy) <= h


# ---------------------------------------------------------------------------
# 위에서 본 전체 지도 그림 (rqt_image_view 로 보는 용도)
# ---------------------------------------------------------------------------
class TopDownView:
    def __init__(self, track, xlim=(-2.0, 2.0), ylim=(-1.3, 1.3), px_per_m=120):
        self.track = track
        self.xlim, self.ylim, self.k = xlim, ylim, px_per_m
        self.W = int((xlim[1] - xlim[0]) * px_per_m)
        self.H = int((ylim[1] - ylim[0]) * px_per_m)
        cols = np.arange(self.W)
        rows = np.arange(self.H)
        self._wx = xlim[0] + (cols[None, :] + 0.5) / px_per_m
        self._wy = ylim[1] - (rows[:, None] + 0.5) / px_per_m
        bg = np.full((self.H, self.W, 3), 235, np.uint8)
        bg[track.is_tape(self._wx, self._wy)] = (30, 30, 30)
        self.bg = bg

    def to_px(self, x, y):
        return (int((x - self.xlim[0]) * self.k), int((self.ylim[1] - y) * self.k))

    def _disc(self, img, x, y, r_m, color):
        mask = (self._wx - x) ** 2 + (self._wy - y) ** 2 <= r_m ** 2
        img[mask] = color

    def draw(self, car, boxes, trail, scan_pts=None):
        img = self.bg.copy()
        for cx, cy, size in boxes:
            h = size / 2
            m = (np.abs(self._wx - cx) <= h) & (np.abs(self._wy - cy) <= h)
            img[m] = (200, 120, 40)
        for tx, ty in trail[::2]:
            c, r = self.to_px(tx, ty)
            if 0 <= r < self.H and 0 <= c < self.W:
                img[r, c] = (40, 120, 230)
        if scan_pts is not None:
            for sx, sy in scan_pts:
                c, r = self.to_px(sx, sy)
                if 1 <= r < self.H - 1 and 1 <= c < self.W - 1:
                    img[r - 1:r + 2, c - 1:c + 2] = (230, 30, 30)
        self._disc(img, car.x, car.y, 0.07, (20, 160, 60))
        # 진행 방향 표시
        for t in np.linspace(0, 0.14, 15):
            c, r = self.to_px(car.x + t * math.cos(car.yaw), car.y + t * math.sin(car.yaw))
            if 0 <= r < self.H and 0 <= c < self.W:
                img[max(r - 1, 0):r + 2, max(c - 1, 0):c + 2] = (250, 250, 0)
        return img


# ---------------------------------------------------------------------------
# 주행 평가 (대회 "보정 주행 기록" 흉내: 시간 + 이탈 횟수)
# ---------------------------------------------------------------------------
class Referee:
    def __init__(self, track):
        self.track = track
        self.reset()

    def reset(self):
        self.t = 0.0
        self.laps = 0
        self.lap_times = []
        self._lap_start = 0.0
        self.off_lane_count = 0
        self._off = False
        self.collision = False
        self._prev_x = None
        self.max_dev = 0.0

    def update(self, car, boxes, dt):
        self.t += dt
        d = float(self.track.centerline_dist(car.x, car.y))
        self.max_dev = max(self.max_dev, d)
        off = d > self.track.lane_width / 2
        if off and not self._off:
            self.off_lane_count += 1
        self._off = off
        # 아래 직선의 x=0 을 +x 방향으로 지나가면 한 바퀴
        if self._prev_x is not None and car.y < 0 and self._prev_x < 0 <= car.x:
            if self.t - self._lap_start > 3.0:
                self.laps += 1
                self.lap_times.append(round(self.t - self._lap_start, 2))
                self._lap_start = self.t
        self._prev_x = car.x
        if any(point_in_box(car.x, car.y, b, margin=0.06) for b in boxes):
            self.collision = True

    def summary(self):
        return {
            "time": round(self.t, 2),
            "laps": self.laps,
            "lap_times": self.lap_times,
            "off_lane_count": self.off_lane_count,
            "penalized_last_lap": (round(self.lap_times[-1] + 5 * self.off_lane_count, 2)
                                   if self.lap_times else None),
            "max_deviation_m": round(self.max_dev, 3),
            "off_lane_now": self._off,
            "collision": self.collision,
        }
