import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zjkj/unitreeGO2_ws/install/go2_teleop_ctrl_keyboard'
