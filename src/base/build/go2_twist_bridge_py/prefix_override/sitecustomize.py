import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zjkj/unitreeGO2_ws/src/base/install/go2_twist_bridge_py'
