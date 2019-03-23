# Winter Project 2019: Hexapodal Gait
**MSR** - Northwestern University

by Andrew Thompson

## Project Description

The aim of this project was (1) to design a modular walking algorithm for a six-legged machine, and (2) for that algorithm to produce a bio-inspired gait.

 Bio-inspired in this context refers to a gait without sharp deviations in its trajectory- an important quality that allows for real system to change direction at high speeds without requiring infinite torque. In its current form, the hexapod has six directions (six gaits) with a variety tunable parameters. These will be discussed in the Software section.

Click through the picture below to see the **demo video!**
<a href="https://youtu.be/c8YuM24-HN4"><img src="images/hexapod.jpg"
 width="720" height="350" border="10"/></a>

All software required to run this code is in the **rpi0w** folder, and is meant to be run in a ROS environment on the Raspberry Pi Zero W used to control this hexapod. The working, final gait is titled: **joy_config_cleaned_bt.py**, and may be run from a properly compiled ROS workspace with:
 >**roslaunch hexapod_motion joy_config_cleaned_bt.py**.

Documents related to the specifics of this project are located in **spreadsheets_and_notes** and **mathematica_files**, respectively. All figures were created in Mathematica. Finally, supporting functions using Adafruit's servo module are included in **python3_supportfunctions**, and are intended to be used as small diagnostic tools.

---

# Hardware

The hexapod is primarily 3D-printed in Tough PLA, held together with metal fasteners.

## Vorpal Hexapod Framework

[Vorpal Robotics Homepage.](http://vorpalrobotics.com/wiki/index.php/Vorpal_Robotics "Vorpal Robotics Homepage")

The body and legs of this hexapod were developed by Vorpal Robotics, a group that originally developed the Vorpal Hexapod as a readily-available learning model funded on Kickstarter. Their models are open-source, and the community around Vorpal is rife with homebrew hexapods (this one included!).

The Vorpal Hexapod has an approximately cylindrical body used to house the electronic components. On this body are six hip slots, which hold one servo a piece. These servos are then connected to a two-piece swiveling 'thigh', which are in turn connected to the servo housing (the knee) of the leg.

The Vorpal Hexapod legs are curved, which allow for an increased degree of stability despite the lack of rigid hip and knee joints. To clarify, the joints at both the hip and knee are fastened to the servo on one side, but remain on floating as they swivel on the other. This compliance in the design can be useful for overcoming small deviations in terrain, but also necessitate extra caution when adding components to the hexapod (weight more readily becomes a limitation).

The legs have two perpendicular revolute joints:
1. the hip joint (x6), which rotates about the *z-axis*.
2. The knee joint (x6), which rotates about the *y-axis*.

## Custom parts

Although the chassis and leg components designed by Vorpal Robotics, the internal scaffolding as well as the 'feet' of this hexapod were designed (in Onshape) specifically for this project.

### Electronics Caddy

![alt text](images/batterycaddy.png "Logo Title Text 1")

The electronics caddy (pictured above) has:
1. A slot to hold the Lithium Ion battery.
2. Mounting holes to secure the caddy to the chassis, as well as mounts for the Raspberry Pi beneath the overhanging slot.
3. A section in the middle to hold a permaboard, accessible to the wiring from the Raspberry Pi above and the servo hardware below (within the chassis).

### Grip Socks
![alt text](images/gripsock.png)

The grip socks were printed out of NinjaFlex PLA, a pliable plastic that tends to print with a coarse grain. By placing these on the feet of each leg, this hexapod was able to generate a better grip, resulting in:
1. Longer feasible strides.
2. Less error in gait cycle due to the aforementioned 'half-floating joints' at the hip and knees.


## BOM

The bill of materials was influenced by the recommended components put forth by Vorpal Robotics and adapted to fit my needs. As follows:

### Battery

1. 4-AA Battery Pack
2. UBEC DC/DC Step-Down (Buck) Converter - 5V @ 3A output
3. DTP 605068 Lithium Ion Battery - 3.7 V; 2000 mAh



### Electronic

1. Raspberry Pi Zero W
2. LiPo 5 V SHIM
3. Adafruit 16-Channel PWM Driver

### Mechanical
1. TowerPro SG92R Servo (x12)
2. PS3 Controller

---

# Software

This project runs via a Raspberry Pi Zero W, and the files are executed from within a ROS workspace. Within the workspace, the scripts are written in Python (2).

Outside of the ROS workspace, a number of '1-off' helper scripts for diagnostics exist, which run Python (3). The control scheme used for input is a PS3 controller via Bluetooth, though scripts are included which work with a wired PS3 controller as well as with keyboard input.

Note, a wired PS3 controller will work with joy_node alone, although to use the wireless functionality afforded by Bluetooth the ps3joy module is required.

## Gait Algorithm

The gait algorithm depends on four initial parameters (listed with their acceptable range):

**stridelength** [0:100]
1. The *stridelength* variable describes the angular sweep from the maximum to the minimum hip angle during one gait cycle.
2. This is essentially the 'width' of a gait pattern.

**kneestance** [20:50]
1. *kneestance* describes the value (in degrees) of the knee at stance.
2. Because of the curve in the foreleg, any angle lower than 20 will cross a 'bounce' point, which will destabilize the hexapod.
3. Any values larger than 50 will raise the foot from the ground, rendering the leg no longer 'in stance.'

**kneerise** [~60:210]
1. The *kneerise* variable is the angle (in degrees) that the knee will reach at it's maximum during a gait's rise cycle.
2. This value minus the kneestance defines the 'height' of a gait pattern.


**velocity** [0.02 minimum]
1. the *velocity* variable determines the frequency at which leg motions will occur. Since the PWM frequency for these servos is 50 Hz, 0.02 is our minimum for reliable motion input.

These four values are used to compute intermediary corner angles, which, when strung together, create an elliptical trajectory. By syncing these elliptical trajectories in different orientations and on different legs in sequence, the hexapod achieves multi-directional motion.

In all cases, the angle values of the hip and knee of each leg are separated into arrays and fed into functions manipulate individual legs in an alternating tripod manner. This means that at any point during the gait, three feet will be in contact with the ground. This is essential for stability.

### Workspace

Since each leg is 2R, the end-effector workspace will trace out a hyperbolic plane.

![alt text](images/gaitlines.png "Logo Title Text 1")

The above image shows an initial plot of the workspace of each leg. Each of the black wireframes represents a position of the leg.

![alt text](images/gaitspace.png "Logo Title Text 1")

And this image shows the topographical plot of that wireframe workspace. Note that the z-axis is a bit skewed in scale. Red regions indicates the full forward extension possible by one leg.

### Directionality

This hexapod is programmed to generate gaits in 6 directions (listed with their gait algorithm primitive):
1. Forward (Normal)
2. Backward (Normal)
3. Left (On-Angle)
4. Right (On-Angle)
5. Turn Left (Turning)
6. Turn Right (Turning)

### Forward vs. Reverse

Each gait pattern is reversible, an integral quality that allows for multi-directional motion.

### Normal Gait

The 'Normal' gait is comprised of 14 steps. These 14 steps include:
 1. Four stride segments (two in contact with the ground and two off the ground) which sum to a length of stridelength/3.
2. Eight corner segments, each with length stridelength/12 and height kneerise/6.
3. Two pure rise components, each with height kneerise/3.

![alt text](images/pjimage.jpg "Logo Title Text 1")

Illustrated above is a half-gait cycle (read left to right, top to bottom), going from tripod 1 in stance and tripod 2 in rise to tripod 1 in rise and tripod 2 in stance.

### Turning Gait

The Turning gaits are perhaps the simplest, with each of the six legs tracing out ellipses in the same direction and at simultaneous rates. The effect is a clockwise (or counter-clockwise) dragging of the body, reorienting the chassis.

### On-Angle Gait

On-Angle gait describes any gait where the sum direction of motion lies in line with a leg. To combat the inherent restrictions of a 2R workspace while maintaining stability, the elliptical gait takes the form of a figure-8.

## Troubleshooting

### Servo Jitter
If you have excessively noisy servos, make sure that the servo you are using is rated for the same PWM frequency that you are using. In the case of a Raspberry Pi, the PWM frequency is set by default, and cannot be changed without some complexity when using standard Python modules like *smbus*. I recommend using a service like ABElectronics' Python (2) PWM package, which enables the user to manually set PWM frequency.

### Force feedback
When using a PS3 controller over Bluetooth, *joy_nod* will not recognize the 'force feedback' (i.e. rumble) functionality of the PS3 controller afforded by *ps3joy*. This will not cause any issues with the control scheme, but the error will look something like:

![alt text](images/error_joystickforcefeedback.png "Logo Title Text 1")

### Socket
When running the launch file multiple times without resetting the Raspberry Pi Zero W, you will likely see the message below.

![alt text](images/error_socket.png "Logo Title Text 1")

This is due to the PS3 controller not desyncing over Bluetooth with the Pi. Hold down the PS button on the controller and wait for ~10 seconds; there will be no verbose response in the terminal, but the controller will resync and work as expected.

---
