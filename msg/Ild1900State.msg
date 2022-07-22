float64 distance # Distance in m
uint32 linearized_distance_raw  # Distance in its digital value format
uint32 sensor_timestamp # Timestamp of measurement in ns (sensor's internal clock)
float64 read_time # Fastcat's cycle read timestamp
uint32 counter # Counter of measurement
uint8 error # Type of measurement error if one is present. Zero means no error.