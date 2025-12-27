import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/revel34/lidar/lidar_arch/install/rover30_slam'
