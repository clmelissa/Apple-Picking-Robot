# Apple-Picking-Robot

### Pin Connection

| Arduino Pin | Sensor |
| ------ | ------ |
| 10 | LED |
| 11 | Button |
| 32 | ULtrasonic |
| analog1 | IR sensor |
| 49,47 | Vertical Motor |
| 53, 52 | Elbow |

### Motor Info
##### Vertical
+- 500 steps/cm
CW  - Move down
CCW  - Move up
Current Driver Setting: 1.5 A Output Current & 1/16 Microstep
##### Shoulder
Kollmorgen Driver w setting: 1.2 A Output Current

### OpenCV Detection
|  | Lower Limit | Upper Limit |
| - | ------ | ------ |
| H | 0 | 14 |
| S | 75 | 255 |
| V | 75 | 250 |

Image size: 640x480
Camera:
* Offset in y: 6 cm
* focal length: 748