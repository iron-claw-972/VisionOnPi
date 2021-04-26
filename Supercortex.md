# Supercortex

This document provides explanations and code samples for Vex actuators and sensors.

# Electrical Guide

## Pragmatics

The electrical stack used to control the Vex electronics is comprised of several key pieces:

* 7.2V Vex Battery
* 5V Buck Converter
* Raspberry Pi 4B
* Adafruit 16 Channel Servo Hat
* Custom Supercortex Board

### 7.2V Vex Battery

<img src="/images/vex/large_robot_battery.png" height="300"/><br>

The 7.2 volt Vex Battery provides power the robot. It is attached using [an adapter](www.google.com) that connects from the white plastic connector to two loose wires, one red and one black. The red positive lead should be connected to the `7.2V` port on the [blue terminal block labeled](www.google.com) `INPUT` located on the Supercortex. The black negative lead should be connected to the `GND` port on the same terminal block.

### 5V Buck Converter

<img src="/images/5v_buck_converter.jpg" height="300"/><br>

This device converts the 7.2V from the battery into a 5V USB port that can be used to power the Raspberry Pi. It should be connected on one end into a designated port on the Supercortex using soldered wires. On the other end it should be connected via a USB A to USB C cable into the power input on the Raspberry Pi.

### Raspberry Pi 4B

<img src="/images/raspberry_pi_4B.jpg" height="300"/><br>

The Raspberry Pi is the brains of the robot. It is the component that runs the code and controls the robot. It is connected through its GPIO Header into both the Adafruit 16 Channel Servo Hat and the Custom Supercortex Board.

### Adafruit 16 Channel Servo Hat

<img src="/images/raspberry_pi_with_adafruit_servo_hat.jpg" height="300"/><br>

This board, sold by Adafruit, has 16 channels of PWM which allow the control of up to 16 motors or servos. It stacks using the GPIO header on top of the Raspberry Pi. For additional rigidity, one can also use M2.5 Hex Standoffs to support this board as it is only supported by the GPIO on one side.

### Custom Supercortex Board

<img src="/images/full_supercortex_stack.jpeg" height="300"/><br>

*The image above is rotated 180 relative to both the Raspberry Pi and Adafruit Servo Hat images above.*

This custom designed and printed board stacks on top of the Servo Hat described above. It uses both the Raspberry Pi GPIO header and a second connector to get the PWM from the Servo Hat.

It includes two terminal blocks on the edge with the indent. The smaller one of these is used to send power to the Adafruit PWM board below. The larger of these acts as the primary power input from the battery for the entire Supercortex stack.

It also includes four 8x3 female connectors on its top. Two of these connectors, labeled PWM provide 16 slots for both Vex motor controllers, Vex servos, and even certain non Vex servos. The connector labeled Digital provides 8 digital inputs for use with Vex Sensors. The analog connector, assuming that the MCP3008 is soldered to the board, provides 8 analog inputs for use with Vex analog sensors.

## Theory

*Preface: This section describes how the different parts of the electrical stack function and communicate. All of this content is abstracted in the final product, meaning that it is not necessary to know in order to have a functional robot.*

TODO: write theory

# Using Sensors and Actuators in Code

## Necessary Libraries

In order to use actuators and sensors with python on the Raspberry Pi a few libraries need to be downloaded. Run the following commands in the terminal:

```shell
sudo pip3 install adafruit-circuitpython-servokit
sudo pip3 install gpiozero
sudo pip3 install Encoder
sudo pip3 install adafruit-blinka
sudo pip3 install adafruit-circuitpython-mcp3xxx
```

## Actuators

There are two major types of actuators provided by Vex:
* Vex Motor
  * This brushed DC motor has an unlimited rotation range. It does not have an encoder. In order to use it, one should connect it to the Vex Motor controller, which in turn should be plugged into a PWM port on the Raspberry Pi Stack.
* Vex Servo
  * A servo motor is generally a heavily geared DC motor with a potentiometer which allows for absolute rotational control. It can rotate within a limited range typically less than 1 rotation. Within this range, the servo can be set to a specific angle. It can be directly connect to a PWM port on the Raspberry Pi Stack.

### Vex Motor Controller

<img src="/images/vex/motor_controller.jpg" height="300"/><br>

In order to use a Vex motor it needs to have the red and black leads connected to the corresponding leads  on the Vex Motor controller.

| Wire Color  | Description        | Where is it connected |
| ----------- | ------------------ | --------------------- |
| Black       | Ground             | gnd                   |
| Orange      | +7.2 V power       | vcc                   |
| White       | PWM control signal | sgl                   |

Here is a code sample for how to control the motor. This sets the motor on PWM slot `0` throttle to `0.0`. The throttle can be changed within the range [-1, 1]. The throttle value is scaled from [-1, 1] to [0, 180] and is fed into the Adafruit servokit. Without scaling, setting the angle to `0` means full reverse, `90` means stop and `180` means full forward. *Important note: Once you set the throttle, the motor will not stop until one either powers down the Raspberry Pi or sets the angle/throttle to `90`/`0.0` respectively.*

```python
from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

while True:
    # set throttle from -1 for full reverse to 1 for full forward
    throttle = 0.0
    # scales output from [-1, 1] to [0, 180] and sets the motor power on PWM slot 0
    kit.servo[0].angle = throttle * 90 + 90  
    time.sleep(0.1)
```

### Servo

<img src="/images/vex/servo_motor.jpg" height="300"/><br>

In order to use the Vex servo it needs to be connected directly to a PWM port.

| Wire Color  | Description        | Where is it connected |
| ----------- | ------------------ | --------------------- |
| Black       | Ground             | gnd                   |
| Orange      | +7.2 V power       | vcc                   |
| White       | PWM control signal | sgl                   |

Here is a code sample for how to control the servo. It sets the angle of the servo to `90` degrees. This value can be changed within the range [0, 120].

```python
from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

while True:
    kit.servo[0].angle = 90
    time.sleep(0.1)
```

## Digital Sensors

There are four types of digital sensors provided by Vex:
* Button
  * A button closes an electrical circuit when it is pressed. It can detect when it has been pushed.
* Line Sensor
  * A line sensor measures the reflected light from a surface underneath it. If it is above a hardware  threshold it is active, otherwise it is not. This can detect dark vs light surfaces underneath a robot, such as lines of dark tape.
* Rotary Encoder
  * An encoder measures the relative rotation of a shaft from when it was powered on. When it is powered on, it starts at 0. If it is rotated in one direction this value will increase and vice-versa. This value has the units of steps, which can be multiplied by a coefficient in order to get more useful units like degrees or centimeters traveled.
* Ultrasonic Rangefinder
  * An ultrasonic rangefinder sends out a pulse of ultrasonic sound and measures the time it takes before it echos back. Given the speed of sound being 343 m/s the distance traveled can be calculated. It allows the measurement of the distance from an object in front of it.

### Vex Button

<img src="/images/vex/bumper_switch.jpg" height="300"/><br>

Connect the Vex button to a digital input port.

| Wire Color  | Description                               | Where is it connected |
| ----------- | ----------------------------------------- | --------------------- |
| Black       | Ground                                    | gnd                   |
| Red         | Not connected                             | vcc                   |
| White       | Connects to ground when button is pressed | sgl                   |

Here is a code sample for how to read from the button. It reads and prints the state of the button connected to GPIO pin `6`.

```python
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

# setup pin 6 as a digital input
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    print(GPIO.input(6) == GPIO.LOW)
    time.sleep(0.1)
```

or

```python
# this code has not been tested
from gpiozero import Button
import time

button = Button(6)

while True:
    print(button.value)
    time.sleep(0.1)
```

[Additional documentation about the button.](https://gpiozero.readthedocs.io/en/stable/api_input.html#button)

### Vex Line Sensor

<img src="/images/vex/line_tracker.jpg" height="300"/><br>

Connect the Vex line sensor to a digital input port.

| Wire Color  | Description | Where is it connected |
| ----------- | ----------- | --------------------- |
| Black       | Ground      | gnd                   |
| Red         | +5V         | vcc                   |
| White       | Signal pin  | sgl                   |

Here is a code sample for how to read from the line sensor. It reads and prints the state of the line sensor connected to GPIO pin `4`.

```python
from gpiozero import LineSensor
import time

sensor = LineSensor(4)

# can be used with callbacks, lambda is basically an inline function definition
sensor.when_line = lambda: print("Enter line")
sensor.when_no_line = lambda: print("Exit line")

while True:
    print(sensor.value)  # needs to be tested
    time.sleep(0.1)
```

[Additional documentation about the line sensor.](https://gpiozero.readthedocs.io/en/stable/api_input.html#linesensor-trct5000)

### Vex Encoder

<img src="/images/vex/optical_shaft_encoder.jpg" height="300"/><br>

Connect the Vex encoder to two digital input ports.

| Wire Color  | Description | Where is it connected |
| ----------- | ----------- | --------------------- |
| Black       | Ground      | gnd                   |
| Red         | +5V         | vcc                   |
| White       | Signal pin  | sgl                   |

Here is a code sample for how to read from the encoder. It reads and prints the state of the encoder connected to GPIO pins `4` and `5`. If you reverse the order of these pins, the encoder will simply measure in the other direction. This is functionally identical to multiplying the encoder output by -1.

```python
import Encoder
import time

# setup encoder with the its 2 pins
enc = Encoder.Encoder(4, 5)

while True:
    print(enc.read())
    time.sleep(0.1)
```

or

```python
# this code has not been tested
from gpiozero import RotaryEncoder
import time

# setup encoder with the its 2 pins
enc = RotaryEncoder(4, 5)

while True:
    print(enc.steps)
    time.sleep(0.1)
```

[Additional documentation about the rotary encoder.](https://gpiozero.readthedocs.io/en/stable/api_input.html#rotaryencoder)

### Ultrasonic Rangefinder

<img src="/images/vex/ultrasonic_rangefinder.jpg" height="300"/><br>

Connect the Vex ultrasonic sensor to two digital input ports.

| Wire Color           | Description                                | Where is it connected |
| -------------------- | ------------------------------------------ | --------------------- |
| Black                | Ground                                     | gnd                   |
| Red                  | +5V                                        | vcc                   |
| Yellow (Input side)  | Trigger pin, sends the ultrasonic pulse    | sgl                   |
| Orange (Output side) | Echo pin, sends signal when echo received  | sgl                   |

Here is a code sample for how to read from the ultrasonic rangefinder. It reads and prints the distance measured by the sensor connected to GPIO pins `31` and `32`. **If you reverse the order of these pins, the ultrasonic sensor may be damaged.** It is important to have the correct trigger and echo GPIO pins when using it.

```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO_TRIGGER = 31
GPIO_ECHO = 32

#wacky thingy: pinout in order to use.

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

while True:
    print ("Measured Distance = %.1f cm" % distance())
    time.sleep(1)
```

or

```python
# this code has not been tested
from gpiozero import DistanceSensor
import time

sensor = DistanceSensor(echo=32, trigger=31)

while True:
    print('Distance: ', sensor.distance * 100)
    time.sleep(0.1)
```

[Additional documentation about the ultrasonic rangefinder.](https://gpiozero.readthedocs.io/en/stable/api_input.html#distancesensor-hc-sr04)

## Analog Sensors

There are two analog Vex sensors:
* Potentiometer
  * These change resistance based on linear or rotational position. In the case of Vex the potentiometer measures the absolute rotation of a shaft in a mechanically limited range.
* Light Sensor
  * These change resistance based on the brightness of light shining on the sensor.

### Reading from MCP3008

<img src="/images/vex/rotary_potentiometer.jpg" height="300"/><br>

<img src="/images/vex/light_sensor.jpg" height="300"/><br>

The MCP3008 chip uses the SPI interface to add 8 analog inputs to the Raspberry Pi. It can be used with many analog sensors with the same code. **If your Supercortex is not equipped with a MCP3008, it will not be able to read analog sensors. Check if the MCP3008 slot is populated.**

| Wire Color  | Description    | Where is it connected |
| ----------- | -------------- | --------------------- |
| Black       | Ground         | gnd                   |
| Red         | +5V            | vcc                   |
| White       | Analog signal  | sgl                   |

The code below reads the value of the analog sensor attached to pin 0 (`P0`) on the MCP and prints it.

```python
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# setup the MCP3008 analog chip on the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D26)
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0 of the 8 available pins within the range [0, 7]
# switch to MCP.P1 for pin 1, etc...
channel0 = AnalogIn(mcp, MCP.P0)

while True:
    print(channel0.value)
    time.sleep(0.1)
```
