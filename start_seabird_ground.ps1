
# Start Seabird Ground Station
$ProjectDir = "C:\Users\Precision 3571\EECS-seaOwlRobotX"
$ComPort = "COM1"
$WslDistro = "Ubuntu-22.04"

# Start Windows ELRS serial/UDP router
Start-Process powershell.exe -ArgumentList @(
    "-NoExit"
    "-Command"
    "Set-Location '$ProjectDir'; py -3.10 .\elrs_windows_router.py --com $ComPort"
)

# Start WSL MAVLink ground receiver
$WslProjectDir = "/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX"

$WslCommand = 'source /opt/ros/humble/setup.bash && exec python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14551 --publish-ros'

wsl.exe -d Ubuntu-22.04 --cd "$WslProjectDir" -- bash -c "$WslCommand"

Write-Host "WSL exited with code: $LASTEXITCODE"
Read-Host "Press Enter to close"


