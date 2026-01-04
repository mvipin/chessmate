# Sensor reading message for various hardware sensors
std_msgs/Header header

# Sensor identification
string sensor_name
uint32 sensor_id

# Sensor type
uint8 HALL_EFFECT = 0
uint8 LIMIT_SWITCH = 1
uint8 ENCODER = 2
uint8 FORCE_SENSOR = 3
uint8 TEMPERATURE = 4
uint8 VOLTAGE = 5
uint8 CURRENT = 6
uint8 sensor_type

# Raw sensor value
float64 raw_value

# Processed/calibrated value
float64 processed_value

# Sensor status
bool is_active
bool is_triggered
bool is_fault

# Calibration data
float64 calibration_offset
float64 calibration_scale

# Sensor location (for position-based sensors)
geometry_msgs/Point position

# Quality metrics
float32 signal_strength
float32 noise_level
