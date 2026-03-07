import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/iris1/swarm_ws/src/orin/install/leader_pkg'
