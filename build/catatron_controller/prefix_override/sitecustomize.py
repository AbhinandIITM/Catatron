import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ajoymathew07/Documents/Envisage/Catatron_ros2/install/catatron_controller'
