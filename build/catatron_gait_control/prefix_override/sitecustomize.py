import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abhinand/Envisage/catatron_ws/install/catatron_gait_control'
