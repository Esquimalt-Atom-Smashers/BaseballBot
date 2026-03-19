package frc.robot.subsystems.candle;

import static frc.robot.subsystems.candle.CANdleConstants.FIRST_LED_INDEX;
import static frc.robot.subsystems.candle.CANdleConstants.LAST_LED_INDEX;
import static frc.robot.subsystems.candle.CANdleConstants.WHITE;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.signals.RGBWColor;

/** IO interface for the Extender (one motor, position controlled). */
public interface CANdleIO {

  @AutoLog
  class CANdleIOInputs {
    AnimationType currentAnimationType = AnimationType.None;
    AnimationType targetAnimationType = AnimationType.None;
    RGBWColor currentColor = WHITE;
    RGBWColor targetColor = WHITE;
    int startLEDIndex = FIRST_LED_INDEX;
    int endLEDIndex = LAST_LED_INDEX;
  }

  /** LED Animation type */
  public enum AnimationType {
      None,
      ColorFlow,
      Fire,
      Larson,
      Rainbow,
      RgbFade,
      SingleFade,
      Strobe,
      Twinkle,
      TwinkleOff,
  }


  /** Update inputs from the hardware. */
  default void updateInputs(CANdleIOInputs inputs) { }

  /** Set the LEDs color. */
  default void setColor(RGBWColor colour) { }

  /** Set the strobe animation of the LEDs */
  default void setAnimationType(AnimationType type) { }

  /** Clear the LEDs */
  default void clear() { }
}
