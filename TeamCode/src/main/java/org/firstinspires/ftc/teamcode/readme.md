Naming Conventions:

variables: variableName_UnitName -> snake_case(camelCase + PascalCase);
functions: functionName -> (camelCase);
files && classes: FileName -> (PascalCase);
Dictionary:

heading: orientation of an object (e.g. where the robot front is facing)
direction: orientation of a vector (e.g. the orientation of the velocity or acceleration)
mode: for motors
state: for subsystems
Default Units:

angle: Radians
distance/position: Meter
time: Seconds
velocity: Meter per Second
acceleration: Meter per Second Squared
angular velocity: Radians per Second
angular acceleration: Radians per Second Squared
Variables Extensions:

wanted: \_W
actual: \_A
processed: \_P (wanted after applying calculations)
Coordinates Systems:

field coordinate system: \_Field_CS
robot coordinate system: \_Robot_CS
Function Returns:

reading values From Devices: read
calculating values: calc
Rates and Changes

delta: difference between cycles
D: derivative
