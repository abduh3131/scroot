# Actuator command produced by the fast planner.

std_msgs/Header header

# Normalized throttle command [0.0, 1.0]
float32 throttle

# Steering angle command in radians. Positive is left.
float32 steering

# Normalized brake command [0.0, 1.0]
float32 brake

# Optional free-form description of what influenced the decision.
string source
