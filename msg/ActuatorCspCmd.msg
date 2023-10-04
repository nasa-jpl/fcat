string name
float64 request_time

float64 target_position

float64 position_offset
float64 velocity_offset
float64 torque_offset_amps

# feed-forward acceleration is only used when interpolation mode is set to IMPLICIT
# setting feed-forward acceleration allows to generate a second order curve from fcat
# acceleration offset is not part of the CSP motor driver spec, and is not sent to
# motor controller
float64 acceleration_offset 

# if CSP setpoints are not received synchronously from calling module, 
# indicate how fcat should update the CSP setpoint to the motor controller
# when it has not received a new command
int16 IMPLICIT=0  # forward interpolation / extrapolation between last setpoint and implied next setpoint
int16 EXPLICIT=1  # 3rd order cubic spline interpolation, backwards interpolation from second to last setpoint received and previous setpoint - introduces latency by two cycles of calling module
int16 interpolation_mode
