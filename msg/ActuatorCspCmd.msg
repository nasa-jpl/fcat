string name
float64 request_time

float64 target_position

float64 position_offset
float64 velocity_offset
float64 torque_offset_amps

# feed-forward acceleration is only used when interpolation mode is set to IMPLICIT;
#
# setting feed-forward acceleration causes fcat to update position and velocity
# setpoints that are forwarded to the actuator using second order extrapolation
#
# acceleration offset is not part of the CSP motor driver spec, and is not sent to
# motor controller. It is used for updating fcat's setpoints only. 
#
# Set acceleration_offset to zero to use first order extrapolation using 
# feed-forward velocity only
float64 acceleration_offset

# if CSP setpoints are not received synchronously from calling module, 
# indicate how fcat should update the CSP setpoint to the motor controller
# when it has not received a new command
int16 IMPLICIT=0  # forward interpolation / extrapolation between last setpoint and implied next setpoint
int16 EXPLICIT=1  # backwards interpolation using two setpoints received from the calling module; it uses third order cubic spline interpolation between setpoints by default, with options for other interpolation orders that are configurable via fcat parameters; explicit interpolation introduces latency by a configurable number of cycles of the calling module
int16 interpolation_mode
