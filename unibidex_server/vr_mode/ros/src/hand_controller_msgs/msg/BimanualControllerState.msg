# Timestamp and coordinate frame
std_msgs/Header         header

# Left controller state
geometry_msgs/Pose      left_pose

float32                 left_index_trigger
float32                 left_hand_trigger

float32[2]              left_stick
bool                    left_stick_pressed
bool                    left_stick_touched

bool[6]                 left_buttons   # A, B, X, Y, Menu, System

# Right controller state
geometry_msgs/Pose      right_pose

float32                 right_index_trigger
float32                 right_hand_trigger

float32[2]              right_stick
bool                    right_stick_pressed
bool                    right_stick_touched

bool[6]                 right_buttons  # A, B, X, Y, Menu, System