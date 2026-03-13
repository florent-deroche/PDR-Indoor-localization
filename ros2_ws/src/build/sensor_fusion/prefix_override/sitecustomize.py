import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/lois5/Documents/PC LOIS GOON/projet/PDR/PDR-Indoor-localization/ros2_ws/src/install/sensor_fusion'
