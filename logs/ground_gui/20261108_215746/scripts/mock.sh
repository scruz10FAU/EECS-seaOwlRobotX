cd '/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX'
export ROBOCOMMAND_ROOT='/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand'
export TEAM_ID='FLAU'
exec python3 -u tests/mock_robocommand_course.py --center=1.28298,103.84663 --size=25,25 --serve --max-alt-hae 60 --no-wait-auto
