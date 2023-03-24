List of things to tune:

- Ports -> For arduino, lidar, and openmv cameras
- Color sensor -> Check RGB values and update them in navSensors/main_code/Sensors.h
- Lidar orientation with respect to robot -> May change behaviour of get_walls service -> Update transform
- PID constants -> Best constants could change depending on surface. Test using navSensors/other/multiplePlots.py
- Kinematics -> If wheels or robot dimensions change, modify kinematics constants in Movement.h
- Openmv: Change thresholds for color detecton, using tools > Machine Vision > Threshold editor
