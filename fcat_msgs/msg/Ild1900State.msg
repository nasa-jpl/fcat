float64 distance_m # Distance in m
float64 intensity # Signal intensity of peak in percentage
uint32 distance_raw  # Distance in its digital value format
uint32 sensor_timestamp_us # Timestamp of measurement in us (sensor's internal clock)
float64 read_time # Fastcat's cycle read timestamp
uint32 counter # Counter of measurement
uint8 error # Type of measurement error if one is present. Zero means no error.