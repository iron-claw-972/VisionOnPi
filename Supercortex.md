# Supercortex

This document provides explanations and code samples for Vex sensors.

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

### Ultrasonic Sensor

Connect the Vex ultrasonic sensor to two digital input ports.

| Wire Color           | Description                                | Where is it connected |
| -------------------- | ------------------------------------------ | --------------------- |
| Black                | Ground                                     | gnd                   |
| Red                  | +5V                                        | vcc                   |
| Yellow (Input side)  | Trigger pin, sends the ultrasonic pulse    | sgl                   |
| Orange (Output side) | Echo pin, sends signal when echo received  | sgl                   |

Here is a code sample for how to read from the ultrasonic sensor. It reads and prints the distance measured by the ultrasonic connected to GPIO pins `31` and `32`. **If you reverse the order of these pins, the ultrasonic sensor may be damaged.** It is important to have the correct trigger and echo GPIO pins when using it.

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

The MCP3008 chip uses the SPI interface to add 8 analog inputs to the Raspberry Pi. It can be used with many analog sensors with the same code.

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