# Vision 1 (draft, L/R interface required)

Existing Fake vision_node is unchanged. Requires the controller team's actual
InspectPositions.srv with presence_l and presence_r; do not generate a substitute.
Install ROS Jazzy cv_bridge, sensor_msgs, python3-opencv and python3-numpy.
Build from ~/cobot2_ws after sourcing Jazzy:

    colcon --log-base log/core-v01 build --base-paths src/core_project \
      --build-base build/core-v01 --install-base install/core-v01 --symlink-install
    source install/core-v01/local_setup.bash
    ros2 launch realsense2_camera rs_launch.py

In another sourced terminal check `ros2 topic list` and calibrate:

    ros2 run core_vision vision1_calibrate --ros-args -p camera_topic:=/camera/camera/color/image_raw

Click physical L (Bolt 2) then R (Bolt 1), not image x order. S saves, C clears.
Calibration is stored outside the repository. Recalibrate if camera moves.

    ros2 run core_vision vision1_node --ros-args -p real_camera:=true -p camera_topic:=/camera/camera/color/image_raw

Set real_camera true ONLY for a live physical camera, never replay or synthetic input.
Default false marks test results unverified. Topic alone cannot prove provenance.
Use controller's actual inspection launch file with mode:=inspection gui:=true.
Confirm controller timeout exceeds wait_seconds (default 1s) plus overhead.
Check both mixed L/R combinations, one-sided UNKNOWN, and stopped camera.
RGB thresholds inherited from uploaded script; uniform ROI rejection is provisional,
not a general occlusion detector. Tune and validate on actual fixture before use.
No ROS build or GUI verification performed here. No fastening service added.
