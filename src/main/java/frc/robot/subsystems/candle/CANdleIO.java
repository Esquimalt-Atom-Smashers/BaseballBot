package frc.robot.subsystems.candle;

import static frc.robot.subsystems.candle.CANdleConstants.kFirstLED;
import static frc.robot.subsystems.candle.CANdleConstants.kEndLED;
import static frc.robot.subsystems.candle.CANdleConstants.AnimationType;

import com.ctre.phoenix6.signals.RGBWColor;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the CANdle.
 */
public interface CANdleIO {

  @AutoLog
  class CANdleIOInputs {
    AnimationType currentAnimationType = AnimationType.None;
    int currentColorRed = 0;
    int currentColorGreen = 0;
    int currentColorBlue = 0;
    int currentColorWhite = 0;
    int startLEDIndex = kFirstLED;
    int endLEDIndex = kEndLED;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(CANdleIOInputs inputs) {}

  /** Set the LEDs color (used by color-aware animations like ColorFlow). */
  default void setColor(RGBWColor colour) {}

  /** Set the LEDs animation to be played. */
  default void setAnimationType(AnimationType type) {}

  /** Clear the LEDs (turn them off). */
  default void clear() {}
}
