"""협동2 체결 루틴 — CLI(fasten_test)와 ROS 2 /fasten Action(core_robot)이 함께 쓰는 공통 코드.

여기에는 로봇 모션 · 힘 제어 · 비전 정렬 · 완료 판정만 있다. 드라이버 건네기(그리퍼 개폐)와
명령행 처리는 fasten_test.py 에, Action 결과 변환은 core_robot/real_fasten.py 에 있다.

호출하는 쪽이 넘길 수 있는 것
  should_abort() -> None | "cancelled" | "stopped"   매 측정 · 모션 전에 불린다 → Interrupted 로 중단
  on_stage(stage, message)                          단계가 바뀔 때 불린다 (Action feedback 용)
  finish(motion="ready" | "retreat" | "hold")       끝낼 때 대기 자세 복귀 / 보기 높이 후퇴 / 그 자리 정지

좌표는 모두 로봇 base 기준(mm, deg). 책상은 base z = -27.1.  [튜닝] 표시 값은 실기에서 정한다.
"""
import csv
import math
import os
import time
from datetime import datetime

from core_common.topics import DSR_ROBOT_ID as ROBOT_ID
ROBOT_MODEL = "m0609"

# ---------------------------------------------------------------- 측정값 (2026-11-26)
DESK_Z = -27.1                 # 책상 높이 (base 기준). 홈에서 플랜지 실측 451.5 - URDF 424.4
TAUGHT_TCP_MM = 208.4          # 볼트 좌표를 잴 때 켜져 있던 TCP
DRIVER_TCP_MM = 369.8          # 플랜지 → 드라이버 끝 (볼트 자세에서 끝단 실측 92.5 mm 로 역산)
DRIVER_TCP_NAME = "Driver_Tip_v1"
TOOL_WEIGHT_NAME = "Tool Weight_1"   # 컨트롤러에 등록된 툴 무게 이름. 없으면 None

# 받침 높이. 11/26 오전 티칭은 스펀지 50 + 지우개 20 + 타공판 5 = 75 mm 위에서 했다.
TAUGHT_SPONGE_MM = 50.0        # 티칭할 때 깔려 있던 스펀지 두께
SPONGE_MM = 0.0                # 지금 스펀지 두께 (스펀지 제거 → 0). 다시 깔면 50.0
SPONGE_DROP = TAUGHT_SPONGE_MM - SPONGE_MM   # 티칭 때보다 볼트가 내려간 양 (50 mm)

# 208.4 TCP 로 티칭한 자세 (x, y, z, a, b, c) — 스펀지 50 mm 위에서 잰 값 그대로.
# b≈180(툴이 수직 아래)이라 z 만 바꾸면 된다. driver_pose() 가 TCP 차이와 SPONGE_DROP 을 뺀다.
BOLT_POSES_TAUGHT = {
    1: [393.42, 69.65, 226.76, 122.92, 179.98, 123.93],
    2: [322.87, 69.65, 226.76, 123.52, 179.98, 123.53],
}
BOARD_TOP_Z = (SPONGE_MM + 20.0 + 5.0) + DESK_Z   # 타공판 윗면 (스펀지 + 지우개 20 + 타공판 5) — 스펀지 없으면 ≈ -2.1

# 대기 자세: 드라이버 끝 ≈ 책상 위 201 mm, (x 269, y 54) 에서 수직 아래.
READY_J = [10.0, -10.0, 80.0, 0.0, 110.0, 0.0]

# ---------------------------------------------------------------- 동작
VIEW_H = 40.0          # [튜닝] 티칭점 위 보기 · 접근 높이 (홈이 화면에 선명하게 보이는 높이)
VIEW_OFFSET_TOOL = [0.0, 0.0]   # [튜닝] 드라이버에 가려 홈이 안 보이면 툴 기준 x, y 로 비켜서 본다 (mm)
START_H = 5.0          # 힘 제어를 시작할 높이 (티칭점 위)
VELJ, ACCJ = 30, 30
VEL_FAST, ACC_FAST = [100, 60], [200, 120]
VEL_SLOW, ACC_SLOW = [20, 30], [50, 60]
VEL_ROT, ACC_ROT = [10, 15], [30, 30]   # 체결 회전 (선속도, 각속도) — 느리게

# ---------------------------------------------------------------- 힘 · 순응 제어
STX = [1000, 1000, 1000, 200, 200, 200]   # [튜닝] xy 를 낮게 → 비트가 홈 중심으로 따라감
PUSH_FZ_CONTACT = -5.0         # [튜닝] 접촉 찾기 힘 (N, 음수 = 아래)
PUSH_FZ_TURN = -12.0           # [튜닝] 돌릴 때 누르는 힘 — 일자 날이 튕겨 나오지 않게 세게
PUSH_RAMP_S = 0.5
DWELL_S = 0.5                  # 힘을 올린 뒤 비트가 홈에 깊이 앉을 때까지 기다림
# 접촉 판정 (Fz 는 영점 뺀 값). 11/26 16:37: 목표 -5 N 인데 센서로는 +2.2 N 만 읽혀 비율 기준(3 N)을 못 넘고
# 닿은 채 시간 초과 → 절대값 기준 + z 멈춤 기준 둘 중 하나면 접촉.
CONTACT_FZ_N = 1.5             # A: |Fz-Fz0| ≥ 1.5 N 이
CONTACT_HOLD_N = 5             #    0.1 초 간격 5 번 연속
STALL_WINDOW_N = 10            # B: 최근 1 초(10 번) 동안
STALL_MM = 0.2                 #    z 변화 < 0.2 mm 이고
STALL_FZ_N = 0.8               #    |Fz-Fz0| ≥ 0.8 N (허공에서 멈춘 것과 구분)
CONTACT_TIMEOUT_S = 60.0         # 힘 제어 하강은 느리다 (11/26: 15 초면 내려가는 도중 중단)
FZ_ABORT = 25.0                # |Fz| 이 값을 넘으면 즉시 중단
# 11/26 16:56: 비트가 십자 구멍 깊이 들어가 접촉 -9.8, 돌리는 중 -13.4 까지 내려감 → 판 기준이 아니라 책상 + 5 mm
Z_FLOOR = DESK_Z + 5.0         # [튜닝] 드라이버 끝이 이보다 내려가면 중단 (≈ -22.1)

# ---------------------------------------------------------------- 홈 찾기 · 체결
SLOT_SYM_DEG = 90.0            # 일자 날 + 십자 홈 → 90° 마다 다시 맞는다
TIGHTEN_SIGN = +1              # 툴 +rz = 위에서 볼 때 시계 방향 (오른나사 조임). 실기에서 확인
SEARCH_STEP_DEG = 15.0
SEAT_DROP_MM = 1.0             # [튜닝] 접촉 높이보다 이만큼 더 내려가면 홈에 들어간 것으로 본다
TIGHTEN_STEP_DEG = 15.0
# 완료 = "토크가 높은데 더는 안 내려감". 11/26 16:46: 12 N 으로 누르며 돌리면 그냥 돌아갈 때도
# |Δtau6| 0.9~1.2 Nm 이고 15° 마다 3~5 mm 씩 내려갔다 → 토크만 보면 다 조이기 전에 완료로 판정됨.
TORQUE_DONE_NM = 1.5           # [튜닝] |tau6 - 영점| 이 이 이상이고
DONE_Z_STALL_MM = 0.5          # [튜닝] 그동안 z 변화가 이보다 작은 상태가
DONE_HOLD_STEPS = 2            #        연속 2 단계(30°) → 체결 완료
MAX_TOTAL_DEG = 360.0 * 8      # 총 회전 상한 → 넘으면 '확인 필요'
MAX_CYCLES = 10

# ---------------------------------------------------------------- 비전
VISION_ITERS = 2               # 재고 돌리고 다시 재기
VISION_FRAMES = 10             # 한 번 잴 때 합치는 프레임 수 (한 장은 conf 가 흔들림 — 11/26 slot_view 실측)
VISION_TOL_DEG = 3.0           # [튜닝] 이보다 작게 어긋나면 정렬 완료
CALIB_TEST_ROT_DEG = 20.0      # 보정 때 방향을 알아내려고 돌려 보는 각도
# 중심 맞추기: 장난감 볼트 홈이 넓어서 날이 홈 한쪽에 걸리는 문제 → 십자 중심을 드라이버 축 픽셀에 맞춘다
CENTER_CAL_MM = 2.0            # 보정 때 툴을 x · y 로 이만큼 움직여 픽셀↔mm 관계(자코비안)를 잰다
CENTER_TOL_MM = 0.3            # [튜닝] 이보다 작게 어긋나면 중심 맞춤 완료
CENTER_MAX_MM = 6.0            # 한 번에 이보다 크게 옮겨야 하면 잘못 잰 것으로 보고 건너뜀
CENTER_ITERS = 3
CALIB_NUDGE_MM = 0.5           # 보정 때 날을 홈 가운데로 옮기는 한 번의 이동량
CALIB_NUDGE_LIFT_MM = 3.0      # 옮기기 전에 살짝 드는 높이

# ---------------------------------------------------------------- J6 · 되감기
J6_LIMIT = 350.0               # M0609 J6 ±360° 에서 여유 10°
J6_REWIND_MARGIN = 10.0

# ---------------------------------------------------------------- 가상(--sim)
SIM_TOTAL_DEG = 720.0

# ---------------------------------------------------------------- 그리퍼
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
GRIP_FORCE = 400               # 0.1 N 단위 → 40 N (RG2 최대)

LOG_DIR = os.path.expanduser("~/cobot2_ws/logs")


class Abort(Exception):
    """안전 조건 위반 또는 진행 불가."""


class DriverRejected(Abort):
    """A motion or force command failed at the Doosan API."""


def require_driver_success(command, result):
    if result is not None and result < 0:
        raise DriverRejected(f'{command} failed ({result})')


class Interrupted(Abort):
    """호출한 쪽의 취소(cancelled) 또는 정지(stopped) 요청으로 멈춤."""

    def __init__(self, kind):
        super().__init__(kind)
        self.kind = kind


RESULT_DONE = "torque_done"        # 체결 완료로 판정
RESULT_SIM_DONE = "sim_done"       # --sim 전용


def image_size(frame):
    """[가로, 세로] 픽셀 — 보정 파일(JSON)에 그대로 저장 · 비교한다."""
    return [int(frame.shape[1]), int(frame.shape[0])]


def driver_pose(bolt_id):
    """208.4 TCP 로 잰 자세를 드라이버 끝 TCP 자세로 바꾸고, 스펀지를 뺀 만큼 내린다
    (툴이 수직 아래라 z 만 내려간다)."""
    p = list(BOLT_POSES_TAUGHT[bolt_id])
    p[2] -= (DRIVER_TCP_MM - TAUGHT_TCP_MM) + SPONGE_DROP
    return p


class Fastener:
    def __init__(self, dr, logger, bolt_id, *, sim=False, vision=True, calib_vision=False,
                 should_abort=None, on_stage=None, log_kind=None, camera_factory=None):
        self.dr = dr
        self.log = logger
        self.sim = sim
        self.bolt_id = bolt_id
        self.bolt = driver_pose(bolt_id)
        self._should_abort = should_abort
        self._on_stage = on_stage
        self._ignore_cancel = False
        self.torque_samples = 0     # 조이는 중 tau6 를 실제로 읽은 횟수 (0 이면 토크 미측정)
        self.stage = ""
        self.force_active = False
        self.total_deg = 0.0
        self.cycle = 0
        self.j6_dir = None          # 툴 +rz 회전 시 J6 증가 방향 (+1/-1), 첫 회전에서 학습
        self.max_tau6 = 0.0
        self.fz0 = 0.0              # 힘 제어 시작 직전(허공) Fz — 툴 무게 미설정으로 생기는 영점 차이를 뺀다
        self.tau0 = 0.0             # 조이기 직전 J6 토크 영점

        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mode = "sim" if self.sim else "real"
        kind = log_kind or ("calib" if calib_vision else "fasten")
        self.stem = os.path.join(LOG_DIR, f"{kind}_{stamp}_bolt{bolt_id}_{mode}")
        self.log_path = self.stem + ".csv"
        self._fp = open(self.log_path, "w", newline="")
        self._csv = csv.writer(self._fp)
        self._csv.writerow(["t", "phase", "cycle", "x", "y", "z", "a", "b", "c",
                            "j6", "fx", "fy", "fz", "tau6", "total_deg", "note"])
        self._fp.flush()
        self._t0 = time.time()

        # 비전
        self.cam = None
        self.calib = None
        self.vision_tag = 0
        use_camera = vision and not self.sim
        if use_camera != (camera_factory is not None):
            self.close()
            raise Abort("vision requires a ROS Image subscriber; simulation or no-vision must omit it")
        if use_camera:
            try:
                from . import slot_vision
                self.sv = slot_vision
                self.cam = camera_factory(self.guard)
                first = self.check_camera()
                if not calib_vision:
                    self.calib = slot_vision.load_calib()
                    if self.calib is None:
                        raise Abort(f"vision_calib 없음 — 먼저 --calib-vision 실행 ({slot_vision.CALIB_PATH})")
                    # ROI · 축 픽셀 · 자코비안은 픽셀 단위 → 해상도가 다르면 엉뚱한 곳을 가리킨다
                    now = {"view_h": VIEW_H, "view_offset_tool": list(VIEW_OFFSET_TOOL), "sponge_mm": SPONGE_MM,
                           "image_size": image_size(first)}
                    if 'camera_topic' in self.calib:
                        now['camera_topic'] = self.cam.topic
                    stale = {k: (self.calib.get(k), v) for k, v in now.items() if self.calib.get(k) != v}
                    if "jac_px_per_mm" not in self.calib:
                        stale["jac_px_per_mm"] = ("없음", "중심 맞추기용")
                    if stale:
                        raise Abort(f"vision_calib 가 지금 설정과 다름 {stale} — --calib-vision 다시 실행")
                    self.log.info(f"비전 보정 불러옴: {self.calib}")
            except (ImportError, RuntimeError) as exc:
                self.close()
                raise Abort(f"camera: {exc}") from exc
            except BaseException:
                self.close()
                raise

    def check_camera(self):
        """Check for a fresh frame when a camera is in use."""
        if self.cam is not None:
            try:
                return self.cam.latest(settle_s=0.0, timeout=3.0)
            except RuntimeError as exc:
                raise Abort(f"camera: {exc}") from exc
        return None

    def check_image_size(self, frames):
        want = (self.calib or {}).get("image_size")
        if want is None:
            return
        for frame in frames:
            if image_size(frame) != want:
                raise Abort(f"camera: 영상 해상도 {image_size(frame)} ≠ 보정 때 {want} — --calib-vision 다시 실행")

    # ------------------------------------------------------------ 중단 확인 · 단계 알림
    def guard(self):
        """호출한 쪽이 취소 · 정지를 요청했으면 다음 측정 · 모션 전에 멈춘다."""
        if self._should_abort is None:
            return
        kind = self._should_abort()
        if kind == "cancelled" and self._ignore_cancel:
            return
        if kind:
            raise Interrupted(kind)

    def set_stage(self, stage, message=""):
        self.stage = stage
        if self._on_stage is not None:
            try:
                self._on_stage(stage, message)
            except Exception as e:   # noqa: BLE001  (알림 실패로 체결을 멈추지 않는다)
                self.log.warn(f"stage 알림 실패: {e}")

    # ------------------------------------------------------------ 상태 읽기 · 로그
    def sample(self, phase, note=""):
        self.guard()
        dr = self.dr
        posx = dr.get_current_posx()[0]
        j6 = dr.get_current_posj()[5]
        f = dr.get_tool_force(dr.DR_BASE)
        f = f if isinstance(f, list) else [math.nan] * 6
        tau = dr.get_external_torque()
        tau6 = tau[5] if isinstance(tau, list) else math.nan
        if not math.isnan(tau6) and phase == "tighten":
            self.max_tau6 = max(self.max_tau6, abs(tau6 - self.tau0))
            self.torque_samples += 1
        self._csv.writerow([round(time.time() - self._t0, 3), phase, self.cycle,
                            *[round(v, 3) for v in posx], round(j6, 2),
                            *[round(v, 3) for v in f[:3]], round(tau6, 4),
                            round(self.total_deg, 1), note])
        self._fp.flush()
        return {"z": posx[2], "j6": j6, "fz": f[2], "tau6": tau6}

    def check_safety(self, s):
        if s["z"] < Z_FLOOR:
            raise Abort(f"z_floor: 드라이버 끝 z={s['z']:.1f} < {Z_FLOOR:.1f}")
        if not math.isnan(s["fz"]) and abs(s["fz"] - self.fz0) > FZ_ABORT:
            raise Abort(f"fz_over: |Fz-Fz0|={abs(s['fz'] - self.fz0):.1f} N > {FZ_ABORT} N")

    # ------------------------------------------------------------ 힘 제어
    def force_on(self, fz):
        if self.sim:
            return
        self.guard()
        if not self.force_active:
            # 허공에서 영점: 11/26 16:31 로그에서 허공 Fz 가 이미 +2.2 N (set_tool 실패) → 접촉 판정이 1 N 만 닿아도 통과했다
            vals = [self.sample("tare")["fz"] for _ in range(5)]
            vals = [v for v in vals if not math.isnan(v)]
            self.fz0 = sum(vals) / len(vals) if vals else 0.0
            self.log.info(f"Fz 영점 {self.fz0:+.2f} N")
            # Cleanup must try release even if the controller accepted the command
            # but the client did not receive a successful response.
            self.force_active = True
            require_driver_success('task_compliance_ctrl', self.dr.task_compliance_ctrl(STX))
            time.sleep(0.2)
        # DR_FC_MOD_REL: 힘 제어 시작 때 센서값을 기준으로 한 상대 힘 (DRL 매뉴얼 5장 — 정확도 위해 권장)
        # 이미 힘 제어 중에 다시 부를 때(5 N → 12 N)도 기준은 시작 때 값으로 유지된다고 보고 쓴다 — 실기 확인 필요
        require_driver_success('set_desired_force', self.dr.set_desired_force(
            [0, 0, fz, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], time=PUSH_RAMP_S, mod=self.dr.DR_FC_MOD_REL))

    def force_off(self):
        if not self.force_active:
            return
        try:
            release_force = self.dr.release_force()
            time.sleep(0.2)
        finally:
            release_compliance = self.dr.release_compliance_ctrl()
        require_driver_success('release_force', release_force)
        require_driver_success('release_compliance_ctrl', release_compliance)
        self.force_active = False

    # ------------------------------------------------------------ 동작 단위
    def move_rel(self, d, ref, vel=VEL_SLOW, acc=ACC_SLOW):
        self.guard()
        require_driver_success('relative movel', self.dr.movel(
            list(d), vel=vel, acc=acc, ref=ref, mod=self.dr.DR_MV_MOD_REL))

    def go_z(self, z_target, vel=VEL_SLOW, acc=ACC_SLOW):
        """현재 xy · 자세 그대로 base z 만 z_target 으로 (툴 회전을 건드리지 않는다)."""
        z = self.dr.get_current_posx()[0][2]
        if abs(z_target - z) > 0.05:
            self.move_rel([0, 0, z_target - z, 0, 0, 0], self.dr.DR_BASE, vel, acc)

    def rotate(self, deg, phase, count=True):
        """툴 Z 축(드라이버 축) 기준 상대 회전. J6 방향을 첫 회전에서 학습한다."""
        self.guard()
        j6_before = self.dr.get_current_posj()[5]
        self.move_rel([0, 0, 0, 0, 0, deg], self.dr.DR_TOOL, VEL_ROT, ACC_ROT)
        if count:
            self.total_deg += abs(deg)
        s = self.sample(phase, f"rot={deg:+.1f}")
        if self.j6_dir is None and abs(s["j6"] - j6_before) > 1.0:
            self.j6_dir = 1 if (s["j6"] - j6_before) * deg > 0 else -1
            self.log.info(f"J6 방향 학습: 툴 +rz → J6 {'+' if self.j6_dir > 0 else '-'}")
        return s

    def j6_has_room(self, tool_deg):
        j6 = self.dr.get_current_posj()[5]
        if self.j6_dir is None:
            return abs(j6) + abs(tool_deg) <= J6_LIMIT
        return abs(j6 + self.j6_dir * tool_deg) <= J6_LIMIT

    # ------------------------------------------------------------ 비전
    def capture(self):
        """보기 자세(필요하면 툴 기준으로 비켜서)에서 VISION_FRAMES 장 찍고 제자리로 돌아온다.
        툴 기준으로 비키므로 볼트는 J6 각도와 상관없이 화면의 같은 자리에 있다."""
        dx, dy = VIEW_OFFSET_TOOL
        moved = abs(dx) > 0.01 or abs(dy) > 0.01
        if moved:
            self.move_rel([dx, dy, 0, 0, 0, 0], self.dr.DR_TOOL)
        try:
            frames = self.cam.frames(n=VISION_FRAMES, settle_s=0.4)
        except RuntimeError as exc:
            raise Abort(f"camera: {exc}") from exc
        if moved:
            self.move_rel([-dx, -dy, 0, 0, 0, 0], self.dr.DR_TOOL)
        self.check_image_size(frames)
        return frames

    def view_and_measure(self, roi, ref=None, frames=None):
        """여러 장을 합쳐 홈 각도를 재고, 표시한 영상을 로그 폴더에 저장한다."""
        if frames is None:
            frames = self.capture()
        if not frames:
            self.sample("vision", "no_frame")
            return None, 0.0, None
        angle, conf, lines, n = self.sv.slot_angle_multi(frames, roi) if roi else (None, 0.0, [], len(frames))
        self.vision_tag += 1
        path = f"{self.stem}_v{self.vision_tag:02d}.jpg"
        import cv2
        last = frames[-1]
        cv2.imwrite(path, self.sv.draw(last, roi, angle, conf, lines, ref) if roi else last)
        cv2.imwrite(path.replace(".jpg", "_raw.jpg"), last)     # 표시 없는 원본 (나중에 다시 분석용)
        self.sample("vision", f"angle={angle} conf={conf:.2f} frames={n} img={os.path.basename(path)}")
        return angle, conf, last

    def measure_center(self, roi, tag="center"):
        frames = self.capture()
        c = self.sv.cross_center(frames, roi) if frames else None
        self.sample(tag, "center=None" if c is None else f"center=({c[0]:.1f},{c[1]:.1f})")
        return c

    def center_by_vision(self):
        """십자 중심을 보정 때의 드라이버 축 픽셀로 옮긴다 (툴 x · y 이동). 맞췄으면 True."""
        import numpy as np
        if self.calib is None or "jac_px_per_mm" not in self.calib:
            return False
        roi = self.calib["roi"]
        axis = np.array(self.calib["axis_px"], float)
        J = np.array(self.calib["jac_px_per_mm"], float)
        for _ in range(CENTER_ITERS):
            c = self.measure_center(roi)
            if c is None:
                self.log.warn("십자 중심 판정 불가 — 중심 맞추기 생략")
                return False
            d = np.linalg.solve(J, axis - np.array(c))      # 툴 x, y (mm)
            n = float(np.hypot(*d))
            self.log.info(f"십자 중심 ({c[0]:.1f}, {c[1]:.1f}) px, 축 ({axis[0]:.1f}, {axis[1]:.1f}) → 툴 "
                          f"x {d[0]:+.2f}, y {d[1]:+.2f} mm")
            if n <= CENTER_TOL_MM:
                return True
            if n > CENTER_MAX_MM:
                self.log.warn(f"중심 오차 {n:.1f} mm > {CENTER_MAX_MM} — 잘못 잰 것으로 보고 생략")
                return False
            self.move_rel([float(d[0]), float(d[1]), 0, 0, 0, 0], self.dr.DR_TOOL)
        return True

    def align_by_vision(self):
        """기준각에 맞춰 툴을 돌린다. 정렬됐으면 True, 판정 불가면 False."""
        if self.calib is None:
            return False
        roi, ref, sign = self.calib["roi"], self.calib["ref_deg"], self.calib["sign"]
        for it in range(VISION_ITERS):
            angle, conf, _ = self.view_and_measure(roi, ref)
            if angle is None:
                self.log.warn(f"비전 판정 불가 (conf={conf:.2f})")
                return it > 0          # 한 번이라도 맞췄으면 정렬된 것으로 본다
            delta = sign * self.sv.wrap90(ref - angle)
            self.log.info(f"홈 각도 {angle:.1f}°, 기준 {ref:.1f}° → 툴 {delta:+.1f}° 회전")
            if abs(delta) <= VISION_TOL_DEG:
                return True
            if not self.j6_has_room(delta):
                alt = delta - math.copysign(SLOT_SYM_DEG, delta)
                if not self.j6_has_room(alt):
                    self.log.warn("J6 여유 없음 — 비전 정렬 생략")
                    return False
                delta = alt
            self.rotate(delta, "vision_align", count=False)
        return True

    # ------------------------------------------------------------ 단계
    def wait_contact(self):
        """힘 제어로 누르며 내려가 접촉 높이를 돌려준다 (힘이 일정 시간 유지되는지로만 판정)."""
        if self.sim:
            self.guard()
            require_driver_success('contact movel', self.dr.movel(self.bolt, vel=VEL_SLOW, acc=ACC_SLOW))
            return self.sample("contact", "sim")["z"]
        hold = 0
        zs = []
        t_end = time.time() + CONTACT_TIMEOUT_S
        while time.time() < t_end:
            s = self.sample("contact")
            self.check_safety(s)
            d = 0.0 if math.isnan(s["fz"]) else abs(s["fz"] - self.fz0)
            hold = hold + 1 if d >= CONTACT_FZ_N else 0
            zs = (zs + [s["z"]])[-STALL_WINDOW_N:]
            stalled = len(zs) == STALL_WINDOW_N and max(zs) - min(zs) < STALL_MM and d >= STALL_FZ_N
            if hold >= CONTACT_HOLD_N or stalled:
                why = "힘" if hold >= CONTACT_HOLD_N else "z 멈춤"
                self.log.info(f"접촉({why}): z={s['z']:.2f}, Fz-Fz0 {s['fz'] - self.fz0:+.1f} N")
                return s["z"]
            time.sleep(0.1)
        raise Abort("no_contact: 제한 시간 안에 접촉하지 못함")

    def seat_search(self, contact_z):
        """홈에 날이 들어갈 때까지 조이는 방향으로 조금씩 돌린다 (최대 90°)."""
        rotated = 0.0
        while rotated < SLOT_SYM_DEG:
            step = TIGHTEN_SIGN * SEARCH_STEP_DEG
            if not self.j6_has_room(step):
                return "j6_limit"
            s = self.rotate(step, "seat_search")
            self.check_safety(s)
            rotated += SEARCH_STEP_DEG
            if self.sim or contact_z - s["z"] >= SEAT_DROP_MM:
                self.log.info(f"홈 맞춤: {rotated:.0f}° 회전, 하강 {contact_z - s['z']:.2f} mm")
                return "seated"
        self.log.warn("90° 안에 하강이 보이지 않음 — 이미 홈에 들어가 있던 것으로 보고 체결 시작")
        return "seat_not_confirmed"

    def tighten(self):
        held = []                   # 조건을 만족한 단계들의 z
        while True:
            if self.total_deg >= MAX_TOTAL_DEG:
                return "max_rotation"
            if self.sim and self.total_deg >= SIM_TOTAL_DEG:
                return "sim_done"
            step = TIGHTEN_SIGN * TIGHTEN_STEP_DEG
            if not self.j6_has_room(step):
                return "j6_limit"
            z_before = self.dr.get_current_posx()[0][2]
            s = self.rotate(step, "tighten")
            self.check_safety(s)
            dtau = math.nan if math.isnan(s["tau6"]) else abs(s["tau6"] - self.tau0)
            dz = z_before - s["z"]
            self.log.info(f"  +{TIGHTEN_STEP_DEG:.0f}° (총 {self.total_deg:.0f}°): z {s['z']:.2f} "
                          f"(이번 {dz:+.2f} mm 하강), |Δtau6| {dtau:.2f} Nm")
            if not math.isnan(dtau) and dtau >= TORQUE_DONE_NM:
                held = (held + [s["z"]])[-DONE_HOLD_STEPS:]
                if len(held) == DONE_HOLD_STEPS and max(held) - min(held) < DONE_Z_STALL_MM:
                    self.log.info(f"체결 완료 판정: {DONE_HOLD_STEPS} 단계 연속 |Δtau6| ≥ {TORQUE_DONE_NM} Nm "
                                  f"이고 z 변화 {max(held) - min(held):.2f} mm")
                    return "torque_done"
            else:
                held = []

    def rewind(self):
        """힘 해제 → 보기 높이로 상승 → J6 를 90° 배수만큼 되감기."""
        self.force_off()
        self.go_z(self.bolt[2] + VIEW_H)
        j6 = self.dr.get_current_posj()[5]
        d_t = (self.j6_dir if self.j6_dir is not None else 1) * TIGHTEN_SIGN   # 조일 때 J6 가 움직이는 방향
        back_limit = -d_t * J6_LIMIT
        room = abs(j6 - back_limit) - J6_REWIND_MARGIN
        n = int(min(room, 2 * J6_LIMIT) // SLOT_SYM_DEG)
        if n <= 0:
            raise Abort(f"rewind: 되감을 여유 없음 (J6={j6:.1f})")
        back = -d_t * SLOT_SYM_DEG * n
        self.log.info(f"되감기: J6 {j6:.1f} → {j6 + back:.1f} ({SLOT_SYM_DEG * n:.0f}°)")
        self.guard()
        require_driver_success('rewind movej', self.dr.movej(
            [0, 0, 0, 0, 0, back], vel=VELJ, acc=ACCJ, mod=self.dr.DR_MV_MOD_REL))
        self.sample("rewind")

    # ------------------------------------------------------------ 전체
    def go_view(self):
        view = list(self.bolt)
        view[2] += VIEW_H
        self.log.info(f"볼트{self.bolt_id} 드라이버 TCP 자세: {[round(v, 2) for v in self.bolt]}")
        self.guard()
        require_driver_success('view movel', self.dr.movel(view, vel=VEL_FAST, acc=ACC_FAST))
        self.sample("view")

    def run(self):
        """체결 한 번. 반환: "torque_done" | "sim_done" | "max_rotation" | "max_cycles".
        안전 조건 위반은 Abort, 취소 · 정지는 Interrupted 로 올라간다."""
        self.check_camera()
        self.set_stage("APPROACH", f"bolt{self.bolt_id} 보기 높이로 이동")
        self.go_view()
        result = "max_cycles"
        while self.cycle < MAX_CYCLES:
            self.cycle += 1
            self.log.info(f"--- 사이클 {self.cycle} ---")
            self.set_stage("ALIGN", f"cycle {self.cycle}: 중심 · 각도 맞추기")
            self.center_by_vision()          # 먼저 드라이버 축을 십자 중심 위로
            aligned = self.align_by_vision()  # 그다음 그 축으로 돌려 각도 맞춤 (중심은 안 움직임)
            self.set_stage("CONTACT", f"cycle {self.cycle}: 힘 제어 하강")
            self.go_z(self.bolt[2] + START_H)
            self.force_on(PUSH_FZ_CONTACT)
            contact_z = self.wait_contact()
            if not aligned:
                self.set_stage("SEAT_SEARCH", f"cycle {self.cycle}: 홈 찾기 회전")
                if self.seat_search(contact_z) == "j6_limit":
                    self.set_stage("REWIND", f"cycle {self.cycle}: J6 되감기")
                    self.rewind()
                    continue
            self.set_stage("TIGHTEN", f"cycle {self.cycle}: 조이기")
            self.force_on(PUSH_FZ_TURN)
            time.sleep(DWELL_S)
            pt = self.sample("push_turn", f"fz_target={PUSH_FZ_TURN}")
            self.tau0 = 0.0 if math.isnan(pt["tau6"]) else pt["tau6"]
            self.log.info(f"돌리기 직전: Fz-Fz0 {pt['fz'] - self.fz0:+.1f} N (목표 {PUSH_FZ_TURN}), tau6 영점 {self.tau0:+.2f} Nm")
            result = self.tighten()
            if result in (RESULT_DONE, RESULT_SIM_DONE, "max_rotation"):
                break
            self.set_stage("REWIND", f"cycle {self.cycle}: J6 되감기")
            self.rewind()     # j6_limit
        return result

    def calibrate_vision(self):
        """날을 홈에 맞춘 상태에서 위로 똑바로 올라가 ROI · 기준각 · 회전 방향을 저장한다."""
        import cv2
        self.check_camera()
        self.go_view()
        self.go_z(self.bolt[2] + START_H)
        self.force_on(PUSH_FZ_CONTACT)
        self.wait_contact()
        nudge = {"i": (CALIB_NUDGE_MM, 0.0), "k": (-CALIB_NUDGE_MM, 0.0),
                 "j": (0.0, CALIB_NUDGE_MM), "l": (0.0, -CALIB_NUDGE_MM)}
        while True:
            k = input("\n[보정] 일자 날이 십자 홈 '가운데'에 제대로 들어갔으면 Enter (이 위치가 중심 기준)\n"
                      f"        r = 15° 더 돌림 | i / k = 툴 x ±{CALIB_NUDGE_MM} mm | j / l = 툴 y ±{CALIB_NUDGE_MM} mm "
                      "(살짝 들었다가 옮겨 다시 내림) > ").strip().lower()
            if k == "r":
                self.rotate(TIGHTEN_SIGN * SEARCH_STEP_DEG, "calib_search", count=False)
            elif k in nudge:
                self.force_off()
                self.move_rel([0, 0, CALIB_NUDGE_LIFT_MM, 0, 0, 0], self.dr.DR_BASE)
                dx, dy = nudge[k]
                self.move_rel([dx, dy, 0, 0, 0, 0], self.dr.DR_TOOL)
                self.force_on(PUSH_FZ_CONTACT)
                self.wait_contact()
            else:
                break
        self.force_off()
        self.go_z(self.bolt[2] + VIEW_H)       # 회전 없이 수직 상승 → 날 방향 유지

        frames = self.capture()
        if not frames:
            raise Abort("calib: 영상 없음")
        roi = self.sv.pick_roi(frames[-1])
        if roi is None:
            raise Abort("calib: ROI 선택 취소")
        ref, conf, _ = self.view_and_measure(roi)
        if ref is None:
            raise Abort(f"calib: 홈 각도 판정 불가 (conf={conf:.2f}) — VIEW_H · 조명 · ROI 확인")
        self.log.info(f"기준각 ref={ref:.1f}° (conf={conf:.2f})")

        self.rotate(CALIB_TEST_ROT_DEG, "calib_test", count=False)
        test, conf2, _ = self.view_and_measure(roi, ref)
        self.rotate(-CALIB_TEST_ROT_DEG, "calib_test", count=False)
        if test is None:
            raise Abort(f"calib: 방향 확인용 판정 불가 (conf={conf2:.2f})")
        change = self.sv.wrap90(test - ref)
        if abs(change) < CALIB_TEST_ROT_DEG * 0.4:
            raise Abort(f"calib: 툴 {CALIB_TEST_ROT_DEG}° 회전에 화면 각도 변화 {change:.1f}° — 카메라가 툴과 같이 도는지 확인")
        # 툴 +20° 에 화면 각도가 change(≈ ±20°) 만큼 변했다 → 화면 변화 = sign × 툴 회전.
        # 화면 각도를 angle → ref 로 옮기려면 툴을 sign × wrap90(ref - angle) 만큼 돌린다.
        sign = 1 if change > 0 else -1

        # 중심 맞추기 보정: 지금 툴 축은 비트가 앉았던 나사 축 바로 위 → 이때 십자 중심 = 축 픽셀
        import numpy as np
        axis = self.measure_center(roi, "calib_center")
        if axis is None:
            raise Abort("calib: 십자 중심 판정 불가")
        cols = []
        for d in ([CENTER_CAL_MM, 0.0], [0.0, CENTER_CAL_MM]):
            self.move_rel([d[0], d[1], 0, 0, 0, 0], self.dr.DR_TOOL)
            c = self.measure_center(roi, "calib_center")
            self.move_rel([-d[0], -d[1], 0, 0, 0, 0], self.dr.DR_TOOL)
            if c is None:
                raise Abort("calib: 툴 이동 후 십자 중심 판정 불가 (ROI 가 너무 작으면 볼트가 밖으로 나간다)")
            cols.append([(c[0] - axis[0]) / CENTER_CAL_MM, (c[1] - axis[1]) / CENTER_CAL_MM])
        J = np.array(cols).T          # 화면 픽셀 변화 = J · 툴 이동(mm)
        det = float(np.linalg.det(J))
        self.log.info(f"축 픽셀 ({axis[0]:.1f}, {axis[1]:.1f}), 자코비안 px/mm {np.round(J, 2).tolist()}, det {det:.2f}")
        if abs(det) < 1.0:
            raise Abort(f"calib: 툴을 {CENTER_CAL_MM} mm 움직였는데 화면 변화가 너무 작음 (det {det:.2f})")
        data = {"roi": roi, "ref_deg": round(ref, 2), "sign": sign,
                "axis_px": [round(axis[0], 2), round(axis[1], 2)],
                "jac_px_per_mm": np.round(J, 4).tolist(),
                "test_change_deg": round(change, 2), "view_h": VIEW_H,
                "view_offset_tool": list(VIEW_OFFSET_TOOL), "sponge_mm": SPONGE_MM, "bolt": self.bolt_id,
                "image_size": image_size(frames[-1]), "camera_topic": self.cam.topic,
                "date": datetime.now().isoformat(timespec="seconds")}
        self.sv.save_calib(data)
        self.log.info(f"비전 보정 저장: {self.sv.CALIB_PATH} → {data}")
        cv2.destroyAllWindows()
        return "calibrated"

    def finish(self, motion="ready"):
        """힘을 풀고 끝낸다. motion:
          "ready"   보기 높이로 올라간 뒤 대기 자세(READY_J)로 (CLI 기본, Action 정상 · 실패 종료)
          "retreat" 보기 높이까지만 수직 상승 (Action 취소)
          "hold"    모션 없음 — 힘만 해제 (정지 요청)
        끝까지 가면 True, 모션이 실패하거나 정지 요청으로 막히면 False."""
        ok = True
        self._ignore_cancel = True          # 취소 뒤 후퇴는 허용, 정지는 계속 막는다
        try:
            self.force_off()
        except Exception as e:   # noqa: BLE001
            ok = False
            self.log.error(f"force_off 실패: {e}")
        if ok and motion in ("ready", "retreat"):
            try:
                z = self.dr.get_current_posx()[0][2]
                if z < self.bolt[2] + VIEW_H:
                    self.go_z(self.bolt[2] + VIEW_H)
                if motion == "ready":
                    self.guard()
                    go_ready(self.dr)
            except Exception as e:   # noqa: BLE001
                ok = False
                self.log.error(f"복귀 실패: {e}")
        self.close()
        return ok

    def close(self):
        try:
            if self.cam is not None:
                camera, self.cam = self.cam, None
                camera.close()
        finally:
            self._fp.close()


def setup_tool(dr, log):
    # 이전 실행이 힘 제어 도중 Ctrl+Z / 강제 종료되면 컨트롤러가 힘 · 순응 제어 상태로 남을 수 있다 → 먼저 해제
    for fn in (dr.release_force, dr.release_compliance_ctrl):
        try:
            fn()
        except Exception as e:   # noqa: BLE001  (켜져 있지 않으면 실패해도 괜찮다)
            log.info(f"{fn.__name__}: {e}")
    if TOOL_WEIGHT_NAME:
        if dr.set_tool(TOOL_WEIGHT_NAME) != 0:
            log.warn(f"set_tool('{TOOL_WEIGHT_NAME}') 실패 — 컨트롤러에 등록된 툴 무게 이름 확인 (힘 값이 틀어질 수 있음)")
    require_driver_success('set manual mode', dr.set_robot_mode(dr.ROBOT_MODE_MANUAL))
    time.sleep(0.5)
    dr.add_tcp(DRIVER_TCP_NAME, [0, 0, DRIVER_TCP_MM, 0, 0, 0])   # 이미 있으면 실패해도 무시
    time.sleep(0.5)
    ok = dr.set_tcp(DRIVER_TCP_NAME)
    time.sleep(0.5)
    require_driver_success('set autonomous mode', dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS))
    time.sleep(0.5)
    if ok != 0:
        raise Abort(f"set_tcp('{DRIVER_TCP_NAME}') 실패")
    log.info(f"TCP: {dr.get_tcp()}  (기대값 {DRIVER_TCP_NAME}, Z {DRIVER_TCP_MM} mm)")


def connect_dsr(node):
    """DSR_ROBOT2 를 node 에 묶어 import 한다. node 는 executor 에 넣지 않는다
    (DSR_ROBOT2 가 서비스 호출마다 spin_until_future_complete(node) 를 직접 돈다).
    클래스 안에서 DR_init.__dsr__id 를 쓰면 이름이 바뀌므로(name mangling) setattr 로 넣는다."""
    import DR_init
    setattr(DR_init, "__dsr__id", ROBOT_ID)
    setattr(DR_init, "__dsr__model", ROBOT_MODEL)
    setattr(DR_init, "__dsr__node", node)
    import DSR_ROBOT2
    return DSR_ROBOT2


def go_ready(dr):
    require_driver_success('READY_J movej', dr.movej(READY_J, vel=VELJ, acc=ACCJ))
