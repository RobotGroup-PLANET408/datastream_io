########################################################################################################################
#
# The ground-truth INS solution message definition
#
# This message contains information is suitable for RobotGVINS
#
########################################################################################################################

# Sequence, timestamp, frame ID
# - Note that the timestamp is defined as GPSTime
Header header       # the timestamp to publish ros message
float64 timestamp   # timestamp

# Solution status
uint8        satnum         # the number of satellites
uint8        satnum_used    # the number of satellites used for positioning
float64[3]   DOP            # HDOP/VDOP/PDOP

# pose and velocity
float64[3]   position_XYZ       # XYZ in ECEF frame (uint:m)
float64[3]   position_LLH       # LLH in ECEF frame (uint:deg/deg/m)
float64[3]   velocity_XYZ       # XYZ in ECEF frame (uint:m/s)
float64[3]   velocity_ENU       # ENU in ENU frame (uint:m/s)
float64[3]   attitude           # Heading/Pitch/Roll in Azimuth (uint:deg)

# # flag
# string       str_header     # the value is $GNLOG
# string       str_end        # the value is *
# string       str_check      # the value is CRC