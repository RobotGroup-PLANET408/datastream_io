
########################################################################################################################
# ___    ___
# \  \  /  /
#  \  \/  /   Copyright (c) Fixposition AG
#  /  /\  \   All rights reserved.
# /__/  \__\
#
########################################################################################################################
#
# GNSS status message definition
#
# This message contains information about the receiver status. It is intended to be published at low
# rate for status display information (e.g. web interface).
#
########################################################################################################################

# Message version
# - Version 2 adds: rel_* fields
int32         VERSION = 2

# Sequence, timestamp, frame ID
# - Note that the header.stamp is derived from gps_wno and gps_tow using leapsecond information from
#   the GNSS receiver. header.stamp is only valid if the time_gps_ok field is true.
Header        header

# Message data validity indicator. If this is false, none of the other fields are valid
bool          msg_valid                # Indicates if the message fields contain valid data

# Atomic time
# - Note that time may not be known yet. Only if time_gps_ok is true the time_gps_wno and time_gps_tow fields
#   contain valid (useful) data.
int16         time_gps_wno             # GPS week number [-]
float64       time_gps_tow             # GPS time of week [s]
bool          time_gps_ok              # time_gps_wno and time_gps_tow contain useful time

# UTC time
# - Note that any of the utc_* fields are only valid (can be trusted) when utc_ok is true
# - Note that time may not be continuous during receiver startup or during longer signal outages.
#   In these cases utc_ok _should_ be false (but there is still a small likelyhood for glitches).
# - Which exact UTC is used depends on the receiver configuration. It can be USNO, NIST, European, SU, etc.)
# - Note that utc_second is a float. Round it properly to display integer seconds!
int16         utc_year                 # UTC year
int8          utc_month                # UTC month (1 = January, ..., 12 = December)
int8          utc_day                  # UTC day of month (1 ... 31)
int8          utc_hour                 # UTC hour of day (0 ... 23)
int8          utc_minute               # UTC minute of hour (0 ... 59)
float32       utc_second               # UTC second of minute (0 ... 60)
bool          utc_ok                   # UTC time fully known, confirmed and unambiguous

# Geodetic position
# - The datum (reference frame) is unknown and depends on the RTK correction service provider. It often is not WGS-84.
#   However, for displaying an approximate position on a map assuming WGS-84 will be close enough in most cases.
float64       pos_lat                  # Latitude [deg]
float64       pos_lon                  # Longitude [deg]
float32       pos_height               # Ellipsoidal height [m]
float32       pos_acc_h                # Horizontal position accuracy estimate [m]
float32       pos_acc_v                # Vertical position accuracy estimate [m]
bool          pos_valid                # pos_* fields contain valid data. See also the fix_* fields below!

# GNSS fix (solution) status
# - Check for fix_ok == true *AND* fix_type for desired quality (e.g. >= FIX_TYPE_RTK_FLOAT)
# - The fix_ok flag is set by the receiver if it meets certain internal criteria and it is set (to false) by our
#   gnss-input node if we think the fix is not trustworthy (e.g. due to contradicting status flags from the receiver)
bool          fix_ok                   # Fix is OK (not masked or otherwise deemed fishy)
int8          fix_type                 # GNSS fix type, one of:
# GNSS fix types (ordered by increasing (typical) fix quality)
int8          FIX_TYPE_UNKNOWN       = 0 # Unknown fix
int8          FIX_TYPE_NOFIX         = 1 # No fix
int8          FIX_TYPE_DRONLY        = 2 # Dead-reckoning only fix
int8          FIX_TYPE_TIME          = 3 # Time only fix (not used on F9P in rover mode)
int8          FIX_TYPE_S2D           = 4 # (Single) 2D fix
int8          FIX_TYPE_S3D           = 5 # (Single) 3D fix
int8          FIX_TYPE_S3D_DR        = 6 # (Single) 3D + dead-reckoning fix (not used on F9P)
int8          FIX_TYPE_RTK_FLOAT     = 7 # RTK float fix (implies S3D fix)
int8          FIX_TYPE_RTK_FIXED     = 8 # RTK fixed fix (implies S3D fix)
int8          FIX_TYPE_RTK_FLOAT_DR =  9 # RTK float fix + dead-reckoning (implies S3D_DR fix)
int8          FIX_TYPE_RTK_FIXED_DR = 10 # RTK fixed fix + dead-reckoning (implies S3D_DR fix)

# Signal level (carrier to noise ratio) information
# - Historgram: number of signals in the respective bin. The bins are:
#    [0]   [1]   [2]    [3]    [4]    [5]    [6]    [7]    [8]    [9]    [10]   [11]
#   0.. / 5.. / 10.. / 15.. / 20.. / 25.. / 30.. / 35.. / 40.. / 45.. / 50.. / 55..oo [dBHz]
# - A "tracked" can be used in navigation. So the number of signals used in nav is always >= signals tracked
#   (compare GnssSig.msg/sig_use enum)
int8[12]      sig_cno_hist_trk         # Number of signals per signal level for signals tracked
int8[12]      sig_cno_hist_nav         # Number of signals per signal level for signals used in navigation
int8          sig_cno_num_trk          # Total number of signals tracked
int8          sig_cno_num_nav          # Total number of signals used in navigation

# Additional navigation solution info
int8          sol_num_sat              # Number of satellites used in solution
int8          sol_num_sat_gps          # Number of GPS satellites used in solution
int8          sol_num_sat_glo          # Number of GLONASS satellites used in solution
int8          sol_num_sat_gal          # Number of Galileo satellites used in solution
int8          sol_num_sat_bds          # Number of BeiDou satellites used in solution
int8          sol_num_sat_other        # Number of other (QZSS, SBAS, ...) satellites used in solution
int8          sol_num_sig              # Number of signals used in solution
int8          sol_num_sig_gps          # Number of GPS signals used in solution
int8          sol_num_sig_glo          # Number of GLONASS signals used in solution
int8          sol_num_sig_gal          # Number of Galileo signals used in solution
int8          sol_num_sig_bds          # Number of BeiDou signals used in solution
int8          sol_num_sig_other        # Number of other (QZSS, SBAS, ...) signals used in solution
int8          sol_num_sig_corr_L1      # Number of used signals with OSR or SSR correction data L1 band
int8          sol_num_sig_corr_L2      # Number of used signals with OSR or SSR correction data L2 band
float32       sol_pdop                 # Position DOP

# Satellite information
int8          sat_num_vis              # Number of satellites theoretically visible (above the horizon)
int8          sat_num_vis_gps          # Number of GPS satellites theoretically visible (above the horizon)
int8          sat_num_vis_glo          # Number of GLONASS satellites theoretically visible (above the horizon)
int8          sat_num_vis_gal          # Number of Galileo satellites theoretically visible (above the horizon)
int8          sat_num_vis_bds          # Number of BeiDou satellites theoretically visible (above the horizon)
int8          sat_num_vis_other        # Number of other (QZSS, SBAS, ...) satellites theoretically visible (above the horizon)

# RF info, see gnss_rxmon.[ch]pp and UBX-MON-RF spec for details
bool          rf_valid                 # rf_* fields valid flag
int32[2]      rf_noise                 # RF noise level (block 1 and 2) [%]
float32[2]    rf_agc                   # RF automatic gain control level (block 1 and 2) [%]
float32[2]    rf_jam                   # RF CW jamming indicator (block 1 and 2) [-]

# Relative position vector (basestation) info
# - The vector is from the basestation to the rover (e.g. rel_pos_n > 0 means rover is North of the base)
float32       rel_pos_n                # Relative position vector North component [m]
float32       rel_pos_e                # Relative position vector East component [m]
float32       rel_pos_d                # Relative position vector Down component [m]
float32       rel_acc_n                # Relative position vector North component accuracy estimate [m]
float32       rel_acc_e                # Relative position vector East component accuracy estimate [m]
float32       rel_acc_d                # Relative position vector Down component accuracy estimate [m]
bool          rel_valid                # rel_* fields valid

# Antenna info, see gnss_rxmon.[ch]pp and UBX-MON-RF spec for details
bool          ant_valid                # ant_* fields valid flag
int8          ant_state                # Antenna supervisor state
int8          ANT_STATE_UNKNOWN  = 0   # State is unknown
int8          ANT_STATE_INIT     = 1   # Antenna supervisor is initialising
int8          ANT_STATE_OK       = 2   # Antenna detected and okay
int8          ANT_STATE_OPEN     = 3   # No antenna detected
int8          ANT_STATE_SHORT    = 4   # Antenna short circuit detected
int8          ant_power                # Antenna power state
int8          ANT_POWER_UNKNOWN  = 0   # Antenna power supply unknown
int8          ANT_POWER_OFF      = 1   # Antenna power supply is off
int8          ANT_POWER_ON       = 2   # Antenna power supply is on

########################################################################################################################