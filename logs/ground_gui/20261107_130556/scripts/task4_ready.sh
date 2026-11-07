source /opt/ros/humble/setup.bash
source '/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand/ros2_ws/install/setup.bash'
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
(ros2 daemon start >/dev/null 2>&1 &)
exec python3 -u -c 'import time, rclpy
from std_msgs.msg import UInt32
rclpy.init(); n = rclpy.create_node('"'"'send_task4_ready'"'"')
p = n.create_publisher(UInt32, '"'"'/seaowls/usv1/task4/ready'"'"', 10)
end = time.time() + 30
while p.get_subscription_count() == 0 and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
if p.get_subscription_count() == 0:
    print('"'"'OCS not found on ROS after 30 s: is it running?'"'"'); raise SystemExit(1)
p.publish(UInt32(data=280061)); print('"'"'reported USV1 ready for seq 280061'"'"')
end = time.time() + 1
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)'
