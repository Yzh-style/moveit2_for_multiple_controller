import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lenigovi/ros2_ws/src/group-a/welding_kinematic_controller/install/welding_kinematic_controller'
