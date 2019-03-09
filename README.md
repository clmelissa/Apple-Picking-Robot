# Apple-Picking-Robot

### Pin Connection

| Arduino Pin | Sensor |
| ------ | ------ |
| 10 | LED |
| 11 | Button |
| 32 | ULtrasonic |
| analog1 | IR sensor |
| 47 | Vertical Motor Dir |
| 49 | Vertical Motor Pulse |
| 53 | Elbow Motor Dir |
| 52 | Elbow Motor Pulse |
| 7 | Shoulder Motor Dir |
| 6 | Shoulder Motor Pulse |
| 24 | Gripper Motor Dir |
| 22 | Gripper Motor Pulse |
| 4 | Twister Motor Dir |
| 3 | Twister Motor Pulse |

### Motor Info
##### Vertical
+- 500 steps/cm </br>
CW  - Move down </br>
CCW  - Move up </br>
Current Driver Setting: 1.5 A Output Current & 1/16 8uMicrostep </br>

##### Shoulder
Kollmorgen Driver w setting: 1.2 A Output Current</br>

##### Twister
The twister motor should not run CCW for more than 180 steps
The twister motor should run CW for around 670 steps

##### Gripper
CW -> Open the gripper

### OpenCV Detection
|  | Lower Limit | Upper Limit |
| - | ------ | ------ |
| H | 0 | 14 |
| S | 75 | 255 |
| V | 75 | 250 |

Image size: 640x480 </br>
Camera:</br>
* Offset in y: 6 cm
* Offset in x: 3.72 cm
* focal length: 748
* Distance between camera 2 and apple is 15.5cm