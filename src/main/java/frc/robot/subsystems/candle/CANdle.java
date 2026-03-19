package frc.robot.subsystems.candle;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** CANdle subsystem: controls the robots LED lights
 * Online example of CANdle subsystem for reference: https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/CANdle/src/main/java/frc/robot/Robot.java
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

  /** Set the LEDs color */
  public void setLEDColor(RGBWColor colour) {
    clearLEDs();
    candleIO.setColor(colour);
  } // End setLEDColor

  /** Clear the LEDs and turn them all off */
  public void clearLEDs() {
    candleIO.clear();
  } // End clearLEDs
}
