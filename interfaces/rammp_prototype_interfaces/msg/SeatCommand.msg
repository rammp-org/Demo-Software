# rammp_prototype_interfaces/msg/SeatCommand.msg
# Published by the GUI to topic: manual_seat_control

# Command type constants
uint8 TILT_FWD        = 1   # Anterior tilt  — front down, rear up  (pitch forward)
uint8 TILT_BACK       = 2   # Posterior tilt — front up,  rear down (pitch backward)
uint8 RAISE           = 3   # Raise entire platform (all actuators up)
uint8 LOWER           = 4   # Lower entire platform (all actuators down)
uint8 LATERAL_LEFT    = 5   # Lateral tilt left  — left side down, right side up
uint8 LATERAL_RIGHT   = 6   # Lateral tilt right — left side up,  right side down
uint8 RESET           = 7   # Move all actuators to their default positions (calibrated zeros)

# Fields
uint8 command         # one of the constants above
