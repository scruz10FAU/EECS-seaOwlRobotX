source /opt/ros/humble/setup.bash
source '/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand/ros2_ws/install/setup.bash'
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
(ros2 daemon start >/dev/null 2>&1 &)
cd '/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX'
exec python3 -u beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14551 --publish-ros --fence-inset-m 1 --ceiling-margin-m 1 --max-alt-limit-hae-m 60 --assume-home-hae-m 0
