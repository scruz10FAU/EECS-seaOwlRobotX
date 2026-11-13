cd '/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX'
export ROBOCOMMAND_ROOT='/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand'
export TEAM_ID='FLAU'
export ROBOCOMMAND_PORT=1914
exec python3 -u tests/mock_robocommand_course.py --corner=1.28173000,103.85391000 --corner=1.28164000,103.85407000 --corner=1.28147000,103.85398000 --corner=1.28158000,103.85381000 --serve --max-alt-hae 60 --no-wait-auto
