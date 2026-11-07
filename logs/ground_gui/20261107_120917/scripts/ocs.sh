source /opt/ros/humble/setup.bash
source '/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand/ros2_ws/install/setup.bash'
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
(ros2 daemon start >/dev/null 2>&1 &)
export ROBOCOMMAND_ROOT='/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand'
export ROBOCOMMAND_HOST='localhost'
export ROBOCOMMAND_PORT='1883'
export TEAM_ID='FLAU'
export VEHICLES='UAV1'
export UAV_GEOFENCE_MAX_ALTITUDE_HAE_M='40'
export UAV_MAX_ALTITUDE_HAE_M='50'
export UAV_PILOT_CALLSIGN='Samantha'
export UAV_REGISTRATION=''
export OCS_DECLARE_MODE='manual'
exec ros2 run seaowls_robocommand seaowls_ocs
