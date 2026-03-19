package frc.robot.subsystems.candle;

import com.ctre.phoenix6.signals.RGBWColor;


/** Constants for the CANdle subsystem */
public class CANdleConstants {
  /** The device id of the CANdle for controlling the LEDs */
  public static final int kDeviceID = 0; // TODO: Get CANdle device ID

  /** LED start index to apply color to */
  public static final int kFirstLED = 0;
  /** LED end index to apply color to */
  public static final int kEndLED = 100; // TODO: Get the proper end value

  /** Maximum brightness of all the LEDs for the brightness scalar */
  public static final double kFullBrightness = 1; 
  /** Minimum brightness of all the LEDs for the brightness scalar */
  public static final double kOffBrightness = 0;
  /** Default brightness of all the LEDs for the brightness scalar */
  public static final double kDefaultBrightness = 0.2;
}
