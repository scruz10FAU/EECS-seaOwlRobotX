source /opt/ros/humble/setup.bash
source '/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand/ros2_ws/install/setup.bash'
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
(ros2 daemon start >/dev/null 2>&1 &)
exec python3 -u -c 'import time, rclpy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node('"'"'restart_drone_mission'"'"')
p = n.create_publisher(String, '"'"'/seaowls/uav1/restart_mission'"'"', 10)
end = time.time() + 30
while p.get_subscription_count() == 0 and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
if p.get_subscription_count() == 0:
    print('"'"'Ground (beacon_mavlink_ground.py) not found on ROS after 30 s'"'"'); raise SystemExit(1)
p.publish(String(data='"'"'090853'"'"')); print('"'"'restart request 090853 sent'"'"')
end = time.time() + 1
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)'
