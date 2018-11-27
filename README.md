# Lidar Lite V3 on RoboRIO for FRC

This code creates a Subsystem that interfaces with a Lidar Lite V3.

## Caveat

I had the Lidar working with this code but then it stopped.

I suspect the 5V supply to the Lidar is inadequate as resources on the
web say it requires up to 500mA at initialization and a capacitor on
the power supply is *required*.  (I was running it off the 5V pin of
the SPI connector with no capacitor.)

Hopefully a clean power hookup will restore it to working but I fear
the Lidar unit may be damaged now somehow.

## Hookup

* Connect the black wire to ground.
* Connect the red wire to 5V (requires 500mA peak and a 1000uF cap)
* Connect the green wire to SCL
* Connect the blue wire to SDA

## Running the Example code

Deploy the project to the RoboRIO and run the Driver Station with SmartDashboard.

Three controls will appear on SmartDashboard:
* lidar_en (initially false)
* lidar_failed (initially false)
* lidar_cm (initially 0)
* lidar_cycles (initially 0)

Enable Teleoperated mode.

Press the A button on the joystick.

The lidar_en state will change to true.  The lidar is enabled now.
The lidar_cycles variable will begin incrementing with each
measurement taken and the lidar_cm indicator will show the distance,
in cm, from the lidar to the first surface the beam hits.

If the lidar_failed flag goes to true, this means an i2c operation
failed when configuring the lidar.

## Using the Example Code

The Lidar subsystem can be enabled or disabled.  The enable() method controls this.

When enabled, the isEnabled() method will return true.

When enabled, if an i2c operation fails, the m_failed flag is set and
subsequent calls to isFailed() method will return true.  This flag is
reset when the lidar is disabled.

If enabled successfully, the subsystem will take distance readings
continuously.  The getDistanceCm() method will read the last value measured.

