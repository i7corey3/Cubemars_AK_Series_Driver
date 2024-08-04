import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/corey/Cubemars_AK_Series_Driver/install/ak_driver_status'
