import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ajoymathew07/Catatron/Catatron/install/catatron_description'
