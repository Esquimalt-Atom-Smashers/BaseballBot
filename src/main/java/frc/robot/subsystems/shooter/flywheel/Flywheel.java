package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TelemetryUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Flywheel subsystem: one motor with onboard velocity control. */
public class Flywheel extends SubsystemBase {

  /** Flywheel state: Idle (low velocity), Charging (ramping to target), At_Speed (ready to shoot) or Manual. */
  public enum State {
    IDLE,
    CHARGING,
    AT_SPEED,
    MANUAL
  } // End State enum

  private final FlywheelIO flywheelIO;
  private final FlywheelIO.FlywheelIOInputs flywheelInputs = new FlywheelIO.FlywheelIOInputs();
  private final String logRoot;
  /** SmartDashboard key prefix (e.g. {@code Flywheel}, {@code SecondSimRobotOutputs/Flywheel}). */
  private final String smartDashboardPrefix;

  private State state = State.IDLE;

  private BooleanSupplier ignoreLimitsSupplier = () -> false;

  private double targetVelocityRadPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(
          TelemetryUtil.roundToTwoDecimals(
              Units.radiansPerSecondToRotationsPerMinute(kDefaultTargetVelocityRadPerSec)));
  private double lastSmartDashboardTargetVelocityRpm =
      TelemetryUtil.roundToTwoDecimals(
          Units.radiansPerSecondToRotationsPerMinute(kDefaultTargetVelocityRadPerSec));

  public Flywheel(FlywheelIO io) {
    this(io, "", "Flywheel");
  } // End Flywheel Constructor

  public Flywheel(FlywheelIO io, String logRoot) {
    this(io, logRoot, "Flywheel");
  } // End Flywheel Constructor

  public Flywheel(FlywheelIO io, String logRoot, String smartDashboardPrefix) {
    flywheelIO = io;
    this.logRoot = logRoot;
    this.smartDashboardPrefix = smartDashboardPrefix;

    SmartDashboard.putNumber(smartDashboardPrefix + "/kP", kP);
    SmartDashboard.putNumber(smartDashboardPrefix + "/kI", kI);
    SmartDashboard.putNumber(smartDashboardPrefix + "/kD", kD);
    SmartDashboard.putNumber(smartDashboardPrefix + "/kV", kV);
    SmartDashboard.putNumber(smartDashboardPrefix + "/kS", kS);
    SmartDashboard.putNumber(
        smartDashboardPrefix + "/TargetVelocityRpm",
        TelemetryUtil.roundToTwoDecimals(Units.radiansPerSecondToRotationsPerMinute(kDefaultTargetVelocityRadPerSec)));
  } // End Flywheel Constructor

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Flywheel/Inputs/MotorConnected", flywheelInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Flywheel/Inputs/VelocityRpm", TelemetryUtil.roundToTwoDecimals(getVelocityRpm()));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Flywheel/Inputs/AppliedVolts", TelemetryUtil.roundToTwoDecimals(flywheelInputs.appliedVolts));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Flywheel/Inputs/SupplyCurrentAmps", TelemetryUtil.roundToTwoDecimals(flywheelInputs.supplyCurrentAmps));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Flywheel/TargetVelocityRpm", TelemetryUtil.roundToTwoDecimals(getTargetVelocityRpm()));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Flywheel/AtTargetVelocity", atTargetVelocity());
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Flywheel/State", state.name());

    if (DriverStation.isDisabled()) {
      flywheelIO.stop();
      return;
    }

    double targetRpmRaw =
        SmartDashboard.getNumber(
            smartDashboardPrefix + "/TargetVelocityRpm",
            Units.radiansPerSecondToRotationsPerMinute(kDefaultTargetVelocityRadPerSec));
    double targetRpm = TelemetryUtil.roundToTwoDecimals(targetRpmRaw);
    if (targetRpm != lastSmartDashboardTargetVelocityRpm) {
      setState(State.CHARGING);
      setTargetVelocityRadPerSec(Units.rotationsPerMinuteToRadiansPerSecond(targetRpm));
    }
    lastSmartDashboardTargetVelocityRpm = targetRpm;

    if (state == State.CHARGING && atTargetVelocity()) {
      state = State.AT_SPEED;
    }

    // If in At_Speed but velocity fell below/outside tolerance, set state to Charging and re-ramp to target velocity.
    if (state == State.AT_SPEED && !atTargetVelocity()) {
      state = State.CHARGING;
    }

    flywheelIO.setTargetVelocity(state == State.IDLE ? kIdleVelocityRadPerSec : getSetpointVelocityRadPerSec());
  } // End periodic

  /** Set the Flywheel state. */
  public void setState(State newState) {
    state = newState;
  } // End setState

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState


  /** Get the current measured velocity in RPM. */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
  } // End getVelocityRpm

  /** Get the current target velocity in RPM. */
  public double getTargetVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(getTargetVelocityRadPerSec());
  } // End getTargetVelocityRpm

  /** Measured velocity in rad/s. */
  public double getVelocityRadPerSec() {
    return flywheelInputs.velocityRadsPerSec;
  } // End getVelocityRadPerSec

  /** Get the current target velocity in rad/s for the current state. */
  public double getTargetVelocityRadPerSec() {
    return state == State.IDLE ? kIdleVelocityRadPerSec : targetVelocityRadPerSec;
  } // End getTargetVelocityRadPerSec

  /** Set target velocity (rad/s) when not Idle; clamped to limits unless limits override is enabled. */
  public void setTargetVelocityRadPerSec(double velocityRadPerSec) {
    targetVelocityRadPerSec = ignoreLimitsSupplier.getAsBoolean() ? velocityRadPerSec : clampTargetVelocity(velocityRadPerSec);
    double targetRpmRounded =
        TelemetryUtil.roundToTwoDecimals(Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadPerSec));
    targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(targetRpmRounded);
    SmartDashboard.putNumber(smartDashboardPrefix + "/TargetVelocityRpm", targetRpmRounded);
    lastSmartDashboardTargetVelocityRpm = targetRpmRounded;
  } // End setTargetVelocityRadPerSec
  

  /** Clamp a target velocity to the enforced min/max range. */
  public double clampTargetVelocity(double velocityRadPerSec) {
    return MathUtil.clamp(velocityRadPerSec, kMinTargetVelocityRadPerSec, kMaxTargetVelocityRadPerSec);
  } // End clampTargetVelocity

  /** Commanded velocity (rad/s) after applying limits when override is off. */
  private double getSetpointVelocityRadPerSec() {
    return ignoreLimitsSupplier.getAsBoolean() ? targetVelocityRadPerSec : clampTargetVelocity(targetVelocityRadPerSec);
  } // End getSetpointVelocityRadPerSec
  
  /** Whether Flywheel is at target velocity within tolerance. */
  public boolean atTargetVelocity() {
    return Math.abs(getVelocityRadPerSec() - getTargetVelocityRadPerSec()) <= kAtTargetVelocityToleranceRadPerSec;
  } // End atTargetVelocity

  /** Whether Flywheel is at target velocity within double tolerance. */
  public boolean atTargetVelocityWithinDoubleTolerance() {
    return Math.abs(getVelocityRadPerSec() - getTargetVelocityRadPerSec()) <= 2.0 * kAtTargetVelocityToleranceRadPerSec;
  } // End atTargetVelocityWithinDoubleTolerance


  /** Set supplier for ignoring limits. */
  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitsSupplier = supplier != null ? supplier : () -> false;
  } // End setIgnoreLimitsSupplier

  /** Step the target velocity by the given amount (rad/s); sets state to Manual. */
  public void stepVelocityRadPerSec(double stepVelocityRadPerSec) {
    // Need to read target before switching to Manual, to get the correct target velocity.
    setTargetVelocityRadPerSec(getTargetVelocityRadPerSec() + stepVelocityRadPerSec);
    state = State.MANUAL;
  } // End stepVelocityRadPerSec
}
