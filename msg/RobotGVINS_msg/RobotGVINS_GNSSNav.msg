
########################################################################################################################
#
# GNSS navigation data definition
#
# This message contains information is suitable for RobotGVINS
#
########################################################################################################################

# Timestamp
# NOTE: the header timstamp is used to publish data
#       and will not be used in GNSS data process
Header header

# Ephemeris data
RobotGVINS_GNSSEph[] ephdata