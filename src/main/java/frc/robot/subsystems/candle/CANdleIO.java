package frc.robot.subsystems.candle;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

/** IO interface for the Extender (one motor, position controlled). */
public interface CANdleIO {

  @AutoLog
  class CANdleIOInputs {
    AnimationType currentAnimationType = AnimationType.None;
    AnimationType targetAnimationType = AnimationType.None;
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
