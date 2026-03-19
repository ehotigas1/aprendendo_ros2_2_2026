import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/unietaravena/Documents/aprendendo_ros2_2_2026/aprendendo_ros2_2_2026/install/probabilistica'
