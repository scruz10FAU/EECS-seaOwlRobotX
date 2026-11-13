cd '/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX'
export ROBOCOMMAND_ROOT='/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand'
export TEAM_ID='FLAU'
export ROBOCOMMAND_PORT=1914
exec python3 -u tests/mock_robocommand_course.py --center=1.2813464,103.8545173 --size=7,7 --serve --max-alt-hae 60 --no-wait-auto
