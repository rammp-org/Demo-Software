# Enum-like string constants (use these names in code)

# Chair control / entry points
string CHAIR_CONTROL_MAIN="ChairControl/main"

# Chair features
string CHAIR_SELFLEVELING_ON="Chair/SelfLeveling/On"
string CHAIR_SELFLEVELING_OFF="Chair/SelfLeveling/Off"

# Curb navigation
string CHAIR_CURB_NAVIGATION="Chair/Curb/Navigation"
string CHAIR_CURB_ASCEND="Chair/Curb/Ascend"
string CHAIR_CURB_DESCEND="Chair/Curb/Descend"
string CHAIR_CURB_CANCEL="Chair/Curb/Cancel"

# Arm control / entry points
string ARM_CONTROL_MAIN="ArmControl/main"

# Arm modes / tasks
string ARM_RETRACT="Arm/Retract"
string ARM_MANUAL="Arm/Manual"
string ARM_OPEN_DOOR="Arm/OpenDoor"
string ARM_ORDER_DRINK="Arm/OrderDrink"
string ARM_DRINKING="Arm/Drinking"
string ARM_CUP_STABLE="Arm/CupStable"
string ARM_CANCEL="Arm/Cancel"

# System
string RESET="Reset"

# The actual payload. Producers must set this to one of the constants above.
string value
