# Control message for GPIO and servo controllers atttached to the FMU
#

Header header

# Device ID: Selects the device to which the commands are forwarded. Following device IDs are valid:
# PX4FMU GPIOs (only GPIO_EXT2 can be used; GPIO_EXT1 is reserved for timesynchronization)
uint8 DEVICE_FMU=0

# PCA9685 PWM controller chip: This device is mounted on the PWR board and connected to the FMU through I2C.
# For different addresses can be set for the chip with two jumpers on the PWR board (A0, A1): 
# Address 0x40 (default)
uint8 DEVICE_PCA9685_0=1
# Address 0x41
uint8 DEVICE_PCA9685_1=2
# Address 0x42
uint8 DEVICE_PCA9685_2=3
# Address 0x43
uint8 DEVICE_PCA9685_3=4

# Device ID:
uint8 device


# Channel: Selects the GPIO or the PWM channel
# For DEVICE_FMU:  {2}
# For DEVICE_PCA9685: [0, 7]
uint8 channel


# Value which should be assigned to the selected channel.
# For a GPIO: {0,1}
# For a Servo pitch: [10.0, 11.0]
# For a PWM Duty cycle: [20.0f, 21.0f]
float32 value


