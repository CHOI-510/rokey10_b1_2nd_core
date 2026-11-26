#!/usr/bin/env python3
"""손목 카메라 영상에서 홈 각도 검출을 실시간으로 확인한다.

  ros2 run cobot_robot slot_view                # 로봇은 움직이지 않음
  ros2 run cobot_robot slot_view --bolt 1       # 대기 자세 → 볼트1 위 보기 높이(VIEW_H)로 이동한 뒤 시작
  ros2 run cobot_robot slot_view --bolt 1 --height 60

키
  화면 판정 = 최근 10 장을 합친 값 (fasten_test 와 같은 방식), 둘째 줄은 한 장 판정 · 각도는 0~90° 로 표시
  p = ROI 고르기 (머리 중심 → 테두리 클릭, Enter)   s = 현재 화면 저장   q / Esc = 종료
  (--bolt 일 때만)  u / d = 5 mm 위 / 아래   [ / ] = 툴 -15° / +15° 회전 (각도가 따라 바뀌는지 확인)
                    i / k = 툴 x +5 / -5 mm   j / l = 툴 y +5 / -5 mm  (볼트를 화면 가운데 · 드라이버에서 멀리)
                    아래로는 보기 높이에서 최대 30 mm 까지만 내려간다.
  비켜선 상태의 회전은 fasten_test 와 똑같이 "볼트 축으로 돌아가 회전 → 다시 비켜서기" 로 한다.
  좋은 자리를 찾으면 터미널에 찍힌 VIEW_H · VIEW_OFFSET_TOOL 값을 fasten_test.py 에 옮겨 적는다.

B-1 PC 에서 send 가 돌고 있어야 하고, b1_camera_udp.py view 창은 닫아야 한다(같은 포트).
로봇을 움직일 때는 터미널 1 에서 s → sodreal(또는 sodvir) 브링업이 떠 있어야 한다.
"""
import argparse
import os
import sys
import time
from collections import deque

import cv2

from cobot_robot import slot_vision as sv
from cobot_robot.udp_camera import UdpCamera

JOG_Z_MM = 5.0
JOG_ROT_DEG = 15.0
JOG_XY_MM = 5.0
MAX_XY_MM = 120.0
MAX_DOWN_MM = 30.0     # 보기 높이에서 이 이상 내려가지 않는다
N_FRAMES = 10          # fasten_test 의 VISION_FRAMES 와 같게 — 화면 판정은 최근 N 장을 합친 값


class Robot:
    """slot_view 용 최소 로봇 제어 — fasten_test 의 상수 · TCP 설정을 그대로 쓴다."""

    def __init__(self, bolt_id, height):
        import rclpy
        import DR_init
        from cobot_robot import fasten_test as ft

        self.ft = ft
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("slot_view", namespace=ft.ROBOT_ID)
        # 클래스 안에서 DR_init.__dsr__id 라고 쓰면 파이썬이 _Robot__dsr__id 로 이름을 바꿔 버린다
        # (name mangling) → setattr 로 정확한 이름에 넣는다.
        setattr(DR_init, "__dsr__id", ft.ROBOT_ID)
        setattr(DR_init, "__dsr__model", ft.ROBOT_MODEL)
        setattr(DR_init, "__dsr__node", self.node)
        import DSR_ROBOT2 as dr
        self.dr = dr
        log = self.node.get_logger()

        ft.setup_tool(dr, log)
        pose = ft.driver_pose(bolt_id)
        self.height = height
        self.view_z = pose[2] + height
        pose[2] = self.view_z
        log.info(f"대기 자세 → 볼트{bolt_id} 위 {height:.0f} mm (드라이버 끝 base z {self.view_z:.1f}, "
                 f"책상 위 {self.view_z - ft.DESK_Z:.1f} mm)")
        dr.movej(ft.READY_J, vel=ft.VELJ, acc=ft.ACCJ)
        dr.movel(pose, vel=ft.VEL_FAST, acc=ft.ACC_FAST)
        self.off = [0.0, 0.0]
        self._shift(list(ft.VIEW_OFFSET_TOOL))     # fasten_test 와 같은 보기 자리에서 시작

    def _shift(self, new_off):
        d = [new_off[0] - self.off[0], new_off[1] - self.off[1]]
        if abs(d[0]) > 0.01 or abs(d[1]) > 0.01:
            self.dr.movel([d[0], d[1], 0, 0, 0, 0], vel=self.ft.VEL_SLOW, acc=self.ft.ACC_SLOW,
                          ref=self.dr.DR_TOOL, mod=self.dr.DR_MV_MOD_REL)
        self.off = list(new_off)

    def jog_xy(self, dx, dy):
        new = [min(max(self.off[0] + dx, -MAX_XY_MM), MAX_XY_MM),
               min(max(self.off[1] + dy, -MAX_XY_MM), MAX_XY_MM)]
        self._shift(new)
        self.report()

    def report(self):
        z = self.dr.get_current_posx()[0][2]
        print(f"VIEW_H = {z - (self.view_z - self.height):.1f}    "
              f"VIEW_OFFSET_TOOL = [{self.off[0]:.1f}, {self.off[1]:.1f}]", flush=True)

    def jog_z(self, dz):
        z = self.dr.get_current_posx()[0][2]
        target = min(max(z + dz, self.view_z - MAX_DOWN_MM), self.view_z + 100.0)
        if abs(target - z) > 0.05:
            self.dr.movel([0, 0, target - z, 0, 0, 0], vel=self.ft.VEL_SLOW, acc=self.ft.ACC_SLOW,
                          ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_REL)
        print(f"z = {target:.1f} (보기 높이 {target - self.view_z:+.1f} mm)", flush=True)
        self.report()

    def rotate(self, deg):
        j6 = self.dr.get_current_posj()[5]
        if abs(j6) + abs(deg) > self.ft.J6_LIMIT:
            print(f"J6={j6:.1f} — 한계 근처라 회전하지 않음", flush=True)
            return
        off = list(self.off)
        self._shift([0.0, 0.0])                    # 볼트 축으로 돌아가서
        self.dr.movel([0, 0, 0, 0, 0, deg], vel=self.ft.VEL_ROT, acc=self.ft.ACC_ROT,
                      ref=self.dr.DR_TOOL, mod=self.dr.DR_MV_MOD_REL)
        self._shift(off)                           # 다시 비켜서기 → 볼트는 화면 같은 자리
        print(f"툴 {deg:+.0f}° → J6 {self.dr.get_current_posj()[5]:.1f}", flush=True)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    try:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(sys.argv)[1:]
    except ImportError:
        argv = sys.argv[1:]
    ap = argparse.ArgumentParser(description="홈 각도 검출 실시간 확인")
    ap.add_argument("--bolt", type=int, choices=[1, 2], help="이 볼트 위 보기 높이로 로봇을 옮긴 뒤 시작")
    ap.add_argument("--height", type=float, default=None, help="티칭점 위 높이 mm (기본 VIEW_H)")
    opts = ap.parse_args(argv)

    calib = sv.load_calib()
    roi = calib["roi"] if calib else None
    ref = calib["ref_deg"] if calib else None
    print(f"calib: {calib}")

    robot = None
    if opts.bolt:
        from cobot_robot import fasten_test as ft
        robot = Robot(opts.bolt, opts.height if opts.height is not None else ft.VIEW_H)

    cam = UdpCamera(unicast=True)
    win = "slot_view"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    last_print = 0.0
    recent = deque(maxlen=N_FRAMES)
    try:
        while True:
            frame = cam.read(timeout=0.5)
            if frame is None:
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                continue
            recent.append(frame)
            if roi:
                one, one_conf, _ = sv.slot_angle(frame, roi)
                angle, conf, lines, n = sv.slot_angle_multi(list(recent), roi)
                vis = sv.draw(frame, roi, angle, conf, lines, ref)
                cv2.putText(vis, f"{n}-frame | single: {sv.to90(one) if one is None else round(sv.to90(one), 1)} "
                            f"({one_conf:.2f})", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                if time.time() - last_print > 1.0:
                    a = "None" if angle is None else f"{sv.to90(angle):5.1f}"
                    o = "None" if one is None else f"{sv.to90(one):5.1f}"
                    print(f"[{n}장] angle={a} conf={conf:.2f}   | 한 장: angle={o} conf={one_conf:.2f}", flush=True)
                    last_print = time.time()
            else:
                vis = frame.copy()
                cv2.putText(vis, "press p to pick ROI", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(win, vis)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                break
            if k == ord("p"):
                picked = sv.pick_roi(frame)
                if picked:
                    roi = picked
                    recent.clear()
                    print(f"roi={roi}")
            elif k == ord("s"):
                path = os.path.expanduser(f"~/cobot2_ws/logs/slot_view_{time.strftime('%H%M%S')}.jpg")
                os.makedirs(os.path.dirname(path), exist_ok=True)
                cv2.imwrite(path, frame)
                print(f"saved {path} (원본)")
            elif robot and k == ord("u"):
                robot.jog_z(JOG_Z_MM)
                recent.clear()
            elif robot and k == ord("d"):
                robot.jog_z(-JOG_Z_MM)
                recent.clear()
            elif robot and k in (ord("i"), ord("k"), ord("j"), ord("l")):
                dx = JOG_XY_MM if k == ord("i") else -JOG_XY_MM if k == ord("k") else 0.0
                dy = JOG_XY_MM if k == ord("j") else -JOG_XY_MM if k == ord("l") else 0.0
                robot.jog_xy(dx, dy)
                recent.clear()
            elif robot and k == ord("["):
                robot.rotate(-JOG_ROT_DEG)
                recent.clear()
            elif robot and k == ord("]"):
                robot.rotate(JOG_ROT_DEG)
                recent.clear()
    finally:
        cam.close()
        cv2.destroyAllWindows()
        if robot:
            robot.close()


if __name__ == "__main__":
    main()
