cd '/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX'
export ROBOCOMMAND_ROOT='/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand'
exec python3 -u tests/mock_robocommand_course.py --center=1.28298,103.84663 --size=25,25 --invalid bad-coords
