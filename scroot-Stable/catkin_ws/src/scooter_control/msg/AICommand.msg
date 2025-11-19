# Defines the final command sent to the motor controller
# It includes a speed value and a steering angle.

# Speed in meters/second.
# Positive for forward, 0 for stop.
float32 speed

# Steering angle in radians.
# 0 is straight, positive is left, negative is right.
float32 steering_angle

# A command string for high-level instructions or status
# e.g., "STOP", "CRUISE", "AVOID_LEFT"
string command
