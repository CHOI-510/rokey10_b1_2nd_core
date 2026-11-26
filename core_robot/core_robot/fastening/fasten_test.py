#!/usr/bin/env python3
"""협동2 1차 코어테스트 — 일자 드라이버로 십자 홈 볼트 체결 (손목 카메라 홈 정렬 포함).

흐름
  1. 대기 자세(READY_J) → 그리퍼를 열고 드라이버를 건네받는다.
  2. 볼트 위 보기 높이(VIEW_H)로 간다.
  3. [사이클]
     a. 비전 정렬: 홈 각도를 재고 기준각(ref)과의 차이만큼 툴을 돌린다 (최대 VISION_ITERS 번).
        판정 불가면 c 단계의 홈 찾기 회전으로 대신한다.
     b. 힘 제어 시작점까지 내려가 접촉 힘(PUSH_FZ_CONTACT)으로 눌러 접촉을 찾는다 (힘만으로 판정).
     c. (비전 정렬이 안 됐을 때만) 15° 씩 최대 90° 돌리며 홈 찾기.
     d. 누르는 힘을 PUSH_FZ_TURN 으로 올리고 DWELL_S 기다린 뒤 조금씩 돌려 조인다.
        J6 토크가 TORQUE_DONE_NM 이상이면 완료.
     e. J6 한계 → 힘 해제, 보기 높이로 상승, J6 를 90° 배수 되감기 → a 로.
  4. 힘 해제 → 보기 높이 → 대기 자세. 결과 출력, CSV · 영상 로그 저장.

실행 (Doosan 브링업과 ROS Image 발행기 실행 후)
  ros2 run core_robot fasten_test --bolt 1 --calib-vision   # 드라이버를 쥘 때마다 1회: ROI · 기준각 · 방향 저장
  ros2 run core_robot fasten_test --bolt 1                  # 체결
  ros2 run core_robot fasten_test --bolt 1 --no-vision      # 비전 없이 (홈 찾기 회전만)
  ros2 run core_robot fasten_test --bolt 1 --sim --no-vision  # 가상: 힘 제어·그리퍼·비전 없이 순서만
  ros2 run core_robot slot_view                             # 로봇 없이 홈 각도 검출만 실시간 확인

기본 영상은 core_common.topics.CAMERA_COLOR를 구독한다. 다른 토픽은 --ros-args -p camera_topic:=<토픽>으로 지정한다.
영상 입력이나 해상도가 바뀌면 같은 영상 입력으로 --calib-vision을 다시 수행한다.

좌표는 모두 로봇 base 기준(mm, deg). 책상은 base z = -27.1.
상수 · 체결 루틴은 fasten_core.py 에 있다 (ROS 2 /fasten Action 과 공용). 이 파일은 명령행 진입점과
드라이버 건네기(그리퍼)만 맡는다. [튜닝] 표시 값은 fasten_core.py 에서 바꾼다.
"""

import argparse
import sys
import time
from contextlib import ExitStack

import rclpy
from rclpy.utilities import remove_ros_args

from core_common import topics

from .fasten_core import (BOLT_POSES_TAUGHT, GRIP_FORCE, GRIPPER_NAME, RESULT_DONE,
                                     RESULT_SIM_DONE, ROBOT_ID, TOOLCHARGER_IP, TOOLCHARGER_PORT,
                                     Abort, Fastener, connect_dsr, go_ready, setup_tool)


def handover(log):
    from robot_control.onrobot import RG
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
    gripper.open_gripper(force_val=GRIP_FORCE)
    time.sleep(1.0)
    input("\n[드라이버 건네기] 드라이버를 그리퍼 사이에 수직으로 대고 Enter 를 누르세요 (손 조심) > ")
    gripper.close_gripper(force_val=GRIP_FORCE)
    time.sleep(0.5)
    while gripper.get_status()[0]:
        time.sleep(0.2)
    grip = gripper.get_status()[1]
    log.info(f"그리퍼 닫힘, grip 감지 = {bool(grip)}")
    if not grip and gripper.connected:
        raise Abort("grip_not_detected: 드라이버를 잡지 못함")
    input("[확인] 드라이버가 수직이고 흔들리지 않으면 Enter (중단은 Ctrl+C) > ")


def ros_camera_factory(topic):
    """비전 입력을 사용할 때만 cv_bridge를 import한다 (구현은 ros_image_camera.py)."""
    from .ros_image_camera import ros_camera_factory as factory
    return factory(topic)


def main(args=None):
    argv = remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser(description="협동2 코어테스트 나사 체결")
    ap.add_argument("--bolt", type=int, choices=sorted(BOLT_POSES_TAUGHT), required=True)
    ap.add_argument("--sim", action="store_true", help="가상: 힘 제어·그리퍼·비전 없이 동작 순서만")
    ap.add_argument("--skip-grip", action="store_true", help="드라이버를 이미 쥐고 있음")
    ap.add_argument("--no-vision", action="store_true", help="비전 정렬 없이 홈 찾기 회전만")
    ap.add_argument("--calib-vision", action="store_true",
                    help="비전 보정만 (ROI · 기준각 · 방향 저장). 드라이버를 다시 쥘 때마다 실행")
    ap.add_argument("--camera-topic", help=argparse.SUPPRESS)
    opts = ap.parse_args(argv)
    if opts.calib_vision and (opts.no_vision or opts.sim):
        ap.error("--calib-vision 은 --no-vision / --sim 과 같이 쓸 수 없음")

    use_camera = not (opts.no_vision or opts.sim)
    if opts.camera_topic is not None and (not opts.camera_topic or not use_camera):
        ap.error("카메라 토픽은 비전을 사용하는 경우에만 설정할 수 있습니다")

    default_topic = (opts.camera_topic or topics.CAMERA_COLOR) if use_camera else ''
    rclpy.init(args=args)
    node = rclpy.create_node("fasten_test", namespace=ROBOT_ID)
    try:
        opts.camera_topic = node.declare_parameter('camera_topic', default_topic).value
        if bool(opts.camera_topic) != use_camera:
            ap.error("비전 체결에는 camera_topic이 필요하며 --sim / --no-vision에서는 비워야 합니다")
        with ExitStack() as stack:
            run(node, opts, stack)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def run(node, opts, stack):
    log = node.get_logger()
    fastener = None
    motion = "hold"
    status, reason = "aborted", ""

    def interrupted():
        return None if rclpy.ok() else "stopped"

    def prepare(camera_factory=None):
        return Fastener(dr, log, opts.bolt, sim=opts.sim, vision=not opts.no_vision,
                        calib_vision=opts.calib_vision, camera_factory=camera_factory,
                        should_abort=interrupted if camera_factory else None)

    try:
        dr = connect_dsr(node)
        if opts.camera_topic:
            factory = stack.enter_context(ros_camera_factory(opts.camera_topic))
            log.info(f"PREPARE: ROS 원본 영상 {opts.camera_topic}")
            fastener = prepare(factory)
        setup_tool(dr, log)
        if fastener is not None:
            fastener.check_camera()
        motion = "ready"
        go_ready(dr)
        if not (opts.sim or opts.skip_grip):
            handover(log)
        if fastener is None:
            fastener = prepare()
        if opts.calib_vision:
            reason = fastener.calibrate_vision()
            status = "calibrated"
        else:
            reason = fastener.run()
            status = {RESULT_DONE: "fastened", RESULT_SIM_DONE: "fastened(sim)"}.get(reason, "needs_check")
    except (Abort, ImportError) as e:
        reason = str(e)
        log.error(f"중단: {reason}")
    except KeyboardInterrupt:
        if opts.camera_topic:
            motion = "hold"
        reason = "keyboard_interrupt"
        log.warn("Ctrl+C — 중단")
    finally:
        if fastener is not None:
            fastener.finish(motion)
            log.info("=" * 60)
            log.info(f"결과: {status}  사유: {reason}")
            log.info(f"볼트{opts.bolt}  사이클 {fastener.cycle}  총 회전 {fastener.total_deg:.0f}°  "
                     f"최대 |tau6| {fastener.max_tau6:.2f} Nm")
            log.info(f"로그: {fastener.log_path}")


if __name__ == "__main__":
    main()
