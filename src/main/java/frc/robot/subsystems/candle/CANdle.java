package frc.robot.subsystems.candle;

import static frc.robot.subsystems.candle.CANdleConstants.AnimationType;
import static frc.robot.subsystems.candle.CANdleConstants.kShootWhenReadyColor;
import static frc.robot.subsystems.candle.CANdleConstants.kIdleColor;
import static frc.robot.subsystems.candle.CANdleConstants.kManualOverrideColor;
import static frc.robot.subsystems.candle.CANdleConstants.kShootWhenReadyScheduledColor;
import static frc.robot.subsystems.candle.CANdleConstants.kManualOverrideAnimation;
import static frc.robot.subsystems.candle.CANdleConstants.kDisabledAnimation;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** CANdle subsystem: controls the robots LED lights
 * @see <a href="https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/CANdle/src/main/java/frc/robot/Robot.java">Online Examples of CANdle</a>
 */
public class CANdle extends SubsystemBase {

  private final CANdleIO candleIO;
  private final CANdleIO.CANdleIOInputs candleIOInputs = new CANdleIO.CANdleIOInputs();

  private Shooter shooter;

  private BooleanSupplier shootWhenReadySupplier = () -> false;
  private BooleanSupplier manualOverrideSupplier = () -> false;

  public CANdle(CANdleIO io) {
    candleIO = io;
  } // End CANdle Constructor

  @Override
  public void periodic() {
    boolean override = manualOverrideSupplier.getAsBoolean();
    boolean shootReady = shootWhenReadySupplier.getAsBoolean();

    // Setting the led color
    if (override) {
      setLEDAnimation(kManualOverrideAnimation);
      setLEDColor(kManualOverrideColor);
      Logger.recordOutput("Subsystems/LED/CANdle/State", "OverrideStrobe");
    } else if (shootReady && shooter != null) {
      setLEDAnimation(AnimationType.None);
      if (shooter.isReadyToShoot()) {
        setLEDColor(kShootWhenReadyColor);
        Logger.recordOutput("Subsystems/LED/CANdle/State", "ShootingWhenReady");
      } else {
        setLEDColor(kShootWhenReadyScheduledColor);
        Logger.recordOutput("Subsystems/LED/CANdle/State", "ShootWhenReadyScheduled");
      }
    } else if (DriverStation.isEnabled()) {
      setLEDAnimation(AnimationType.None);
      setLEDColor(kIdleColor);
      Logger.recordOutput("Subsystems/LED/CANdle/State", "EnabledIdle");
    } else {
      setLEDAnimation(kDisabledAnimation);
      Logger.recordOutput("Subsystems/LED/CANdle/State", "DisabledRainbow");
    }

    candleIO.updateInputs(candleIOInputs);

    Logger.recordOutput("Subsystems/LED/CANdle/Inputs/CurrentAnimationType", candleIOInputs.currentAnimationType.name());
    Logger.recordOutput(
        "Subsystems/LED/CANdle/Inputs/CurrentColorHex",
        String.format(
            "#%02X%02X%02X%02X",
            candleIOInputs.currentColorRed,
            candleIOInputs.currentColorGreen,
            candleIOInputs.currentColorBlue,
            candleIOInputs.currentColorWhite));
    Logger.recordOutput("Subsystems/LED/CANdle/Inputs/StartLEDIndex", candleIOInputs.startLEDIndex);
    Logger.recordOutput("Subsystems/LED/CANdle/Inputs/EndLEDIndex", candleIOInputs.endLEDIndex);
  } // End periodic

  /** Supplier: true while {@link frc.robot.commands.ShootWhenReadyCommand} (or equivalent) is active. */
  public void setShootWhenReadySupplier(BooleanSupplier supplier) {
    shootWhenReadySupplier = supplier != null ? supplier : () -> false;
  } // End setShootWhenReadySupplier

  /** Supplier: true when driver or operator manual override should flash red. */
  public void setManualOverrideSupplier(BooleanSupplier supplier) {
    manualOverrideSupplier = supplier != null ? supplier : () -> false;
  } // End setManualOverrideSupplier

  /** Set shooter subsystem */
  public void setShooter(Shooter shooter) {
    this.shooter = shooter;
  } // End setShooter

  /** Set the LEDs color to be used in the current animation */
  public void setLEDColor(RGBWColor colour) {
    candleIO.setColor(colour != null ? colour : new RGBWColor());
  } // End setLEDColor

  /** Set the LEDs animation to be played */
  public void setLEDAnimation(AnimationType type) {
    candleIO.setAnimationType(type != null ? type : AnimationType.None);
  } // End setLEDAnimation

  /** Clear the LEDs and turn them all off */
  public void clearLEDs() {
    candleIO.clear();
  } // End clearLEDs
}
