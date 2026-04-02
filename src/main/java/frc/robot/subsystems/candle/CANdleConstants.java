package frc.robot.subsystems.candle;

import com.ctre.phoenix6.signals.RGBWColor;

/** Constants for the CANdle subsystem */
public class CANdleConstants {
  /** The device id of the CANdle for controlling the LEDs */
  public static final int kDeviceID = 26;

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

  /** Solid orange: default enabled idle indication */
  public static final RGBWColor kOrange = new RGBWColor(255, 165, 0);
  /** Solid green: shoot-when-ready active */
  public static final RGBWColor kGreen = new RGBWColor(0, 255, 0);
  /** Strobe red: driver or operator manual override */
  public static final RGBWColor kRed = new RGBWColor(255, 0, 0);

  /**
  * LED Animation type
  */
  public enum AnimationType {
      None,
      ColorFlow,
      Rainbow,
      Strobe,
      SingleFade,
      RgbFade,
      Twinkle,
      TwinkleOff,
      Larson,
      Fire,
  }
}
