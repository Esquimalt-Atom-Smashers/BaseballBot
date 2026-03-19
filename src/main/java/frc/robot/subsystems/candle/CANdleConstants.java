package frc.robot.subsystems.candle;

import com.ctre.phoenix6.signals.RGBWColor;


/** Constants for the CANdle subsystem */
public class CANdleConstants {
  /** The device id of the CANdle for controlling the LEDs */
  public static final int CANDLE_LED_DEVICE_ID = 0; // TODO: Get CANdle device ID

  /** LED start index to apply color to */
  public static final int FIRST_LED_INDEX = 0;
  /** LED end index to apply color to */
  public static final int LAST_LED_INDEX = 100; // TODO: Get the proper end value

  /** Maximum brightness of all the LEDs for the brightness scalar */
  public static final double LED_FULL_BRIGHTNESS = 1; 
  /** Minimum brightness of all the LEDs for the brightness scalar */
  public static final double LED_OFF_BRIGHTNESS = 0;
  /** Default brightness of all the LEDs for the brightness scalar */
  public static final double LED_DEFAULT_BRIGHTNESS = 0.2;

  /** The color red */
  public static final RGBWColor RED = new RGBWColor(255, 0, 0);
  /** The color green */
  public static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  /** The color blue */
  public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  /** The color purple */
  public static final RGBWColor PURPLE = new RGBWColor(128, 0, 128);
  /** The color white */
  public static final RGBWColor WHITE = new RGBWColor(255, 255, 255);
  /** Turns the LED brightness to 0, setting it off */
  public static final RGBWColor OFF = new RGBWColor(0, 0, 0);
}
