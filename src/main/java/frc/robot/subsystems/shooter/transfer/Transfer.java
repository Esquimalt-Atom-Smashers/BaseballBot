package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.TelemetryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Transfer subsystem: Staging (low voltage, stop when sensor tripped (optional)) or Shooting (high voltage). */
public class Transfer extends SubsystemBase {
  private static final String kTargetVoltageKey = "Transfer/TargetVoltage";
  private static final double kTargetVoltageWidgetEpsilon = 1e-3;

  /** Transfer state: Idle, Staging (slow pre-load), Shooting, or Manual. */
  public enum State {
    IDLE,
    STAGING,
    SHOOTING,
    MANUAL
  } // End State enum

  private final TransferIO transferIO;
  private final TransferIO.TransferIOInputs transferInputs = new TransferIO.TransferIOInputs();
  private final String logRoot;

  private State state = State.IDLE;
  private boolean colourSensorEnabled = false;
  private boolean ballStaged = false;
  private double targetVoltage = TelemetryUtil.roundToTwoDecimals(kIdleVoltage);
  private BooleanSupplier ignoreLimitsSupplier = () -> false;
  private double lastTargetVoltageDashboardWrite = Double.NaN;

  public Transfer(TransferIO io) {
    this(io, "");
  } // End Transfer Constructor

  public Transfer(TransferIO io, String logRoot) {
    transferIO = io;
    this.logRoot = logRoot;
    SmartDashboard.putNumber(kTargetVoltageKey, TelemetryUtil.roundToTwoDecimals(targetVoltage));
  } // End Transfer Constructor

  @Override
  public void periodic() {
    transferIO.updateInputs(transferInputs);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Transfer/Inputs/MotorConnected", transferInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Transfer/Inputs/AppliedVolts", TelemetryUtil.roundToTwoDecimals(transferInputs.appliedVolts));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Transfer/Inputs/SupplyCurrentAmps", TelemetryUtil.roundToTwoDecimals(transferInputs.supplyCurrentAmps));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Transfer/Inputs/ColorSensorTripped", transferInputs.colorSensorTripped);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Transfer/TargetVolts", TelemetryUtil.roundToTwoDecimals(getTargetVoltage()));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Transfer/BallStaged", ballStaged);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Transfer/State", state.name());

    if (DriverStation.isDisabled()) {
      transferIO.stop();
      lastTargetVoltageDashboardWrite = Double.NaN;
      return;
    }

    boolean useSmartDashboardTarget = ignoreLimitsSupplier.getAsBoolean();
    if (useSmartDashboardTarget) {
      double dashboardVolts = SmartDashboard.getNumber(kTargetVoltageKey, targetVoltage);
      boolean dashboardTargetEdited =
          !Double.isNaN(lastTargetVoltageDashboardWrite)
              && Math.abs(dashboardVolts - lastTargetVoltageDashboardWrite) > kTargetVoltageWidgetEpsilon;
      if (dashboardTargetEdited) {
        setTargetVoltage(dashboardVolts);
        if (state == State.IDLE) {
          state = State.MANUAL;
        }
      }
    }
    targetVoltage = TelemetryUtil.roundToTwoDecimals(targetVoltage);
    SmartDashboard.putNumber(kTargetVoltageKey, targetVoltage);
    lastTargetVoltageDashboardWrite = targetVoltage;

    // Set the Transfer voltage based on the current state.
    switch (state) {
      case IDLE:
        transferIO.stop();
        break;
      case STAGING:
        if (colourSensorEnabled && transferInputs.colorSensorTripped) {
          transferIO.stop();
          ballStaged = true;
          state = State.IDLE;
        } else {
          transferIO.setVoltage(targetVoltage, ignoreLimitsSupplier.getAsBoolean());
        }
        break;
      case SHOOTING:
      case MANUAL:
        transferIO.setVoltage(targetVoltage, ignoreLimitsSupplier.getAsBoolean());
        break;
      default:
        transferIO.stop();
        break;
    }
  } // End periodic

  /** Set state to Idle (motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
    setTargetVoltage(kIdleVoltage);
  } // End setIdleState

  /** Set state to Staging (slow pre-load; stop when colour sensor tripped (optional)). */
  public void setStagingState() {
    state = State.STAGING;
    setTargetVoltage(kStagingVoltage);
  } // End setStagingState

  /** Set state to Shooting (fast load); clears ballStaged. */
  public void setShootingState() {
    state = State.SHOOTING;
    ballStaged = false;
    setTargetVoltage(kShootingVoltage);
  } // End setShootingState

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState

  /** Set the target voltage (rounded to two decimals). */
  public void setTargetVoltage(double volts) {
    targetVoltage = TelemetryUtil.roundToTwoDecimals(volts);
  } // End setTargetVoltage

  /** Get the current target voltage. */
  public double getTargetVoltage() {
    return targetVoltage;
  } // End getTargetVoltage

  
  /** Set supplier for ignoring limits. */
  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitsSupplier = supplier != null ? supplier : () -> false;
  } // End setIgnoreLimitsSupplier

  /** Step the target voltage by the given amount. */
  public void stepVoltage(double stepVoltage) {
    boolean wasIdle = getState() == State.IDLE;
    state = State.MANUAL;
    boolean ignoreLimits = ignoreLimitsSupplier.getAsBoolean();
    if (wasIdle) {
      setTargetVoltage(ignoreLimits 
        ? MathUtil.clamp(stepVoltage, -Constants.kNominalVoltage, Constants.kNominalVoltage)
        : MathUtil.clamp(stepVoltage, -kMaxVoltage, kMaxVoltage));
    }
    else {
      double stepTargetVoltage = getTargetVoltage() + stepVoltage;
      setTargetVoltage(ignoreLimits 
        ? MathUtil.clamp(stepTargetVoltage, -Constants.kNominalVoltage, Constants.kNominalVoltage)
        : MathUtil.clamp(stepTargetVoltage, -kMaxVoltage, kMaxVoltage));
    }
    if (Math.abs(getTargetVoltage() - kIdleVoltage) < 1e-6) {
      setIdleState();
    }
  } // End stepVoltage

  /** True when in Staging and colour sensor was tripped (ball at transfer). */
  public boolean isBallStaged() {
    return ballStaged;
  } // End isBallStaged
}
