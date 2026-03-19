package frc.robot.subsystems.candle;

import static frc.robot.subsystems.candle.CANdleConstants.kFirstLED;
import static frc.robot.subsystems.candle.CANdleConstants.kEndLED;
import static frc.robot.subsystems.candle.CANdleConstants.AnimationType;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;

/**
 * IO interface for the CANdle
 */
public interface CANdleIO {

    @AutoLog
    class CANdleIOInputs {

        AnimationType currentAnimationType = AnimationType.None;
        AnimationType targetAnimationType = AnimationType.None;
        RGBWColor currentColor = new RGBWColor(Color.kWhite);
        RGBWColor targetColor = new RGBWColor(Color.kWhite);
        int startLEDIndex = kFirstLED;
        int endLEDIndex = kEndLED;
    }
    
    /**
     * Update inputs from the hardware.
     */
    default void updateInputs(CANdleIOInputs inputs) { }

    /**
     * Set the LEDs color.
     */
    default void setColor(RGBWColor colour) { }

    /**
     * Set the strobe animation of the LEDs
     */
    default void setAnimationType(AnimationType type) { }

    /**
     * Clear the LEDs
     */
    default void clear() { }
}
