Header header  # standard ROS message header

string GPGGA
string GNGLL
string GNGNS
string GNGST
string GPGSV
string GLGSV
string GNHDT
string GNRMC
string GNVTG
string GNZDA
string GNROT
string GNGMP

float64 longitude
float64 latitude

float64 x_coordinate
float64 y_coordinate

float64 altitude
float64 altitude_orthometric

int8 fix_state
string fix_state_str

float64 horizental_precision
float64 lat_std
float64 lon_std
float64 altitude_std

int8 number_of_sat

bool heading_valid
float64 heading_true
float64 heading_magnet
float64 speed_knot
float64 speed_km
string GNVTG_mode

