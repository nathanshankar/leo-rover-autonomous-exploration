import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nathan/leo-rover-autonomous-exploration/install/leo_v4'
