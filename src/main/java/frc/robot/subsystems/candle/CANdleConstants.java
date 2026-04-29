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
  public static final RGBWColor kIdleColor = new RGBWColor(255, 40, 0, 255);
  /** Solid green: shoot-when-ready active and can shoot */
  public static final RGBWColor kShootWhenReadyColor = new RGBWColor(0, 255, 0, 255);
  /** Solid blue: shoot-when-ready active but cant shoot */
  public static final RGBWColor kShootWhenReadyScheduledColor = new RGBWColor(0, 0, 255, 255);
  /** Strobe red: driver or operator manual override */
  public static final RGBWColor kManualOverrideColor = new RGBWColor(255, 0, 0, 255);

  /** Strobe animation: manual override is on (Driver or operator) */
  public static final AnimationType kManualOverrideAnimation = AnimationType.SingleFade;
  /** Rainbow animation: robot is disabled */
  public static final AnimationType kDisabledAnimation = AnimationType.Rainbow;

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
