
# Start Seabird Ground Station
$ProjectDir = "C:\Users\Precision 3571\EECS-seaOwlRobotX\elrs_windows_routing"
$ComPort = "COM1"
$WslDistro = "Ubuntu-22.04"

# ROS 2 + robocommand workspace (seaowls_interfaces). Spaces in the path are
# backslash-escaped rather than quoted so this survives being nested inside
# the Start-Process -Command string below.
$RosSetup = 'source /opt/ros/humble/setup.bash && source /mnt/c/Users/Precision\ 3571/seaowls-robotx-robocommand/ros2_ws/install/setup.bash'

# Start Windows ELRS serial/UDP router
Start-Process powershell.exe -ArgumentList @(
    "-NoExit"
    "-Command"
    "Set-Location '$ProjectDir'; py -3.10 .\elrs_windows_router.py --com $ComPort --wsl-log 'C:\Users\Precision 3571\EECS-seaOwlRobotX\tests\elrs_testing\flight_test1\uav_mavlink_relay.csv'"
)

# Start RoboCommand OCS (needs the MQTT broker at ROBOCOMMAND_HOST, default localhost:1883)
$OcsCommand = "$RosSetup && exec ros2 run seaowls_robocommand seaowls_ocs"
Start-Process powershell.exe -ArgumentList @(
    "-NoExit"
    "-Command"
    "wsl.exe -d $WslDistro -- bash -ic '$OcsCommand'"
)

# Start WSL MAVLink ground receiver
$WslProjectDir = "/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX"

$WslCommand = "$RosSetup && exec python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14551 --publish-ros --log '/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX/tests/elrs_testing/flight_test1/wsl_output.csv'"

# bash -i: load ~/.bashrc (venv/pymavlink, ROS/DDS env) the same as a manual
# WSL terminal -- non-interactive bash -c skips it.
wsl.exe -d $WslDistro --cd "$WslProjectDir" -- bash -ic "$WslCommand"

Write-Host "WSL exited with code: $LASTEXITCODE"
Read-Host "Press Enter to close"


