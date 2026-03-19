package frc.robot.subsystems.candle;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.candle.CANdleConstants.AnimationType;

/** CANdle subsystem: controls the robots LED lights
 * @see <a href="https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/CANdle/src/main/java/frc/robot/Robot.java">Online Examples of CANdle</a>
 */
public class CANdle extends SubsystemBase {

  private final CANdleIO candleIO;
  private final CANdleIO.CANdleIOInputs candleIOInputs = new CANdleIO.CANdleIOInputs();

  public CANdle(CANdleIO io) {
    candleIO = io; 
  }

  @Override
  public void periodic() {
    candleIO.updateInputs(candleIOInputs);

    if (DriverStation.isDisabled()) {
      candleIO.clear();
      return;
    }
  } // End periodic

  /** Set the LEDs color to be used in the current animation */
  public void setLEDColor(RGBWColor colour) {
    clearLEDs();
    candleIO.setColor(colour);
  } // End setLEDColor

  /** Set the LEDs animation to be played */
  public void setLEDAnimation(AnimationType type) {
    clearLEDs();
    candleIO.setAnimationType(type);
  } // End setLEDAnimation

  /** Clear the LEDs and turn them all off */
  public void clearLEDs() {
    candleIO.clear();
  } // End clearLEDs
}
