package frc.robot.subsystems.candle;

import static frc.robot.subsystems.candle.CANdleConstants.AnimationType;
import static frc.robot.subsystems.candle.CANdleConstants.kShootWhenReadyColor;
import static frc.robot.subsystems.candle.CANdleConstants.kIdleColor;
import static frc.robot.subsystems.candle.CANdleConstants.kManualOverrideColor;
import static frc.robot.subsystems.candle.CANdleConstants.kShootWhenReadyScheduledColor;
import static frc.robot.subsystems.candle.CANdleConstants.kShootWhenReadyTempDisabledColor;
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

  private BooleanSupplier autoShootEnabledSupplier = () -> false;
  private BooleanSupplier autoShootTempDisabledSupplier = () -> false;
  private BooleanSupplier manualOverrideSupplier = () -> false;

  public CANdle(CANdleIO io) {
    candleIO = io;
  } // End CANdle Constructor

  @Override
  public void periodic() {
    candleIO.updateInputs(candleIOInputs);

    String ledState = setCustomLEDColors();

    Logger.recordOutput("Subsystems/LED/CANdle/State", ledState);
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

  /** Sets the color of the LEDs depending on what state the robot is in  */
  private String setCustomLEDColors() {
    boolean override = manualOverrideSupplier.getAsBoolean();
    boolean shootReady = autoShootEnabledSupplier.getAsBoolean();
    boolean shootTempDisabled = autoShootTempDisabledSupplier.getAsBoolean();

    AnimationType targetAnimation = AnimationType.None;
    RGBWColor targetColor = new RGBWColor(0, 0, 0, 0);
    String ledState;

    // Setting LED color/animation
    if (override) {
      // OVERRIDE STATES
      targetAnimation = kManualOverrideAnimation;
      targetColor = kManualOverrideColor;
      ledState = "OverrideStrobe";
    } else if (shootReady && shooter != null) {
      // SHOOTER STATES
      if (shooter.isReadyToShoot()) {
        targetColor = kShootWhenReadyColor;
      ledState = "ShootingWhenReady";
      } else if (shootTempDisabled) {
        targetColor = kShootWhenReadyTempDisabledColor;
        ledState = "ShootWhenReadyTempDisabled";
      } else {
        targetColor = kShootWhenReadyScheduledColor;
        ledState = "ShootWhenReadyTempDisabled";
      }
    } else if (DriverStation.isEnabled()) {
      // IDLE
      targetColor = kIdleColor;
      ledState = "EnabledIdle";
    } else {
      // DISABLED
      targetAnimation = kDisabledAnimation;
      ledState = "DisabledRainbow";
    }

    setLEDAnimation(targetAnimation);
    setLEDColor(targetColor);
    return ledState;
  } // End setCustomLEDColors

  /** Supplier: true while {@link frc.robot.commands.autoShootEnabled} (or equivalent) is active. */
  public void setAutoShootEnabledSupplier(BooleanSupplier supplier) {
    autoShootEnabledSupplier = supplier != null ? supplier : () -> false;
  } // End setAutoShootEnabledSupplier

  /** Supplier: true when driver or operator manual override should flash red. */
  public void setManualOverrideSupplier(BooleanSupplier supplier) {
    manualOverrideSupplier = supplier != null ? supplier : () -> false;
  } // End setManualOverrideSupplier

  /** Supplier: true while in the trench  */
  public void setAutoShootTempDisabledSupplier(BooleanSupplier supplier) {
    autoShootTempDisabledSupplier = supplier != null ? supplier : () -> false;
  } // End setAutoShootTempDisabledSupplier

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
