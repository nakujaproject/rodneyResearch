# Research on Avionics

Avionics is the electrical system necessary for flight and driven by software to tell the rocket where it should go.
Avionics system could also be used to gather data from the sorrounding and send it to the base station for later analysis.

### Minor definition
**Flight computer** - This is the computer that handles the rocket for the first 8 mins from lift off to separation.
**Engine Flight controller** - This ensures engine is performing to the required optimum conditions
**Booster avionics** - This provides 80% control of the rocket after boosting stage.

### Altimeter
The altimeter is the computer component that contains basically:
- RF tracking beacons
- RF telemetry beacons
- miniature audio and optical location sensors
- multichannel data acquisition
- GPS system
- Interial measuring unit - IMU - MPU6050
- Transponder
- Power - rechargeable LiPo battery

The control can either be passive or active.

Active control deals with sensors that measure various parameters and through some on board process module, provide control signals. For example fin control or gimbled thrust control. This sensors get data e.g acceleration, altitude, position and orientation.

Passive control depends on the geometry of the body. For example the stationary fins which provide stablitiy and orientation.

Motor create lift offthat produces torque about the centre of gravity which makes the rocket fly. For N1 we will be guding it straight upwards using the launch lug. This will create a trajectory which by the time burn out of the motor reaches we release the parachute for recovery.

### Sensors
Some of the sensors include:
1. BMP180

![BMP180 image](https://cdn-shop.adafruit.com/1200x900/1603-03.jpg)

- VIN voltage input
- GND - common ground
- SCL (Serial Clock) clock signal which synchronize the data transfer
- SDA (Serial Data) data signal

Some of the characteristics of BMP180 are:
- Interface - I2C Pressure range 300-1100HPa
- Resolution 0.25m
- Range - upto 30000ft

The BMP180 was designed to accurately measure atmospheric pressure. Atmospheric pressure varies with both weather and altitude; you can measure both of these using this sensor. 

2. MPU6050
![MPU 6050 image](https://components101.com/admin/sites/default/files/components/MPU6050-Module.jpg)
![MPU 6050 pinout](https://components101.com/admin/sites/default/files/component_pin/MPU6050-Pinout.png)

	
- Vcc Provides power for the module, can be +3V to +5V. Typically +5V is used
- Ground Connected to Ground of system
- Serial Clock (SCL) Used for providing clock pulse for I2C Communication
- Serial Data (SDA) Used for transferring Data through I2C communication
- Auxiliary Serial Data (XDA) Can be used to interface other I2C modules with MPU6050. It is optional
- Auxiliary Serial Clock (XCL) Can be used to interface other I2C modules with MPU6050. It is optional
- AD0 If more than one MPU6050 is used a single MCU, then this pin can be used to vary the address

The MPU6050 is a Micro Electro-Mechanical Systems (MEMS) which consists of a 3-axis Accelerometer and 3-axis Gyroscope inside it. This helps us to measure acceleration, velocity, orientation, displacement and many other motion related parameter of a system or object. This module also has a (DMP) Digital Motion Processor inside it which is powerful enough to perform complex calculation and thus free up the work for Microcontroller.

The module also have two auxiliary pins which can be used to interface external IIC modules like an magnetometer, however it is optional. Since the IIC address of the module is configurable more than one MPU6050 sensor can be interfaced to a Microcontroller using the AD0 pin. This module also has well documented and revised libraries available hence it’s very easy to use with famous platforms like Arduino. So if you are looking for a sensor to control motion for your RC Car, Drone, Self balancing Robot, Humanoid, Biped or something like that then this sensor might be the right choice for you.


### Actuators
The actuators for N1 rocket will be ther servo motor.

#### Workings
You can control the servo motor by sending a series of pulses to the signal line. A conventional analog servo motor expects to receive a pulse roughly every 20 milliseconds (i.e., the signal should be 50Hz).

The length of the pulse determines the position of the servo motor.

If the pulse is high for 1ms, then the servo angle will be zero. If the pulse is high for 1.5ms, then the servo will be at its central position. If the pulse is high for 2ms, then the servo will operate at 180 degrees. Pulses ranging between 1ms and 2ms will move the servo shaft through the full 180 degrees of its travel.

#### Pin out
![Servo image](https://i.imgur.com/4LFDthm.png)

- GND (black or brown wire) is a common ground for both the motor and logic.
- 5V (red wire) is a positive voltage that powers the servo.
- Control (orange or yellow wire)is input for the control system.

For example let’s use SG90 Micro Servo Motor. It runs on 4.8-6VDC (5V Typical) and can rotate approximately 180 degrees (90 in each direction)..
It consumes around 10mA at idle and 100mA to 250mA when moving, so we can power it up through 5-volt output on the dev kit.
If you have a servo that consumes more than 250mA, consider using a separate power supply for your servo.
