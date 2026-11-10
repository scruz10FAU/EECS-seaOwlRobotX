cd '/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX'
export ROBOCOMMAND_ROOT='/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand'
export TEAM_ID='FLAU'
export ROBOCOMMAND_PORT=1883
exec python3 -u tests/mock_robocommand_course.py --center=1.3170485,103.6279227 --size=25,25 --serve --max-alt-hae 60 --no-wait-auto
