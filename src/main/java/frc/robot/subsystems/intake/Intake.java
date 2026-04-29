package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.TelemetryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem: field-to-storage transfer. */
public class Intake extends SubsystemBase {
  private static final String kTargetVoltageKey = "Intake/TargetVoltage";
  private static final double kTargetVoltageWidgetEpsilon = 1e-3;

  /** Intake state: Idle, Intaking (pull in), Reversing (spit out), or Manual. */
  public enum State {
    IDLE,
    INTAKING,
    REVERSING,
    MANUAL
  } // End State enum

  private final IntakeIO intakeIO;
  private final IntakeIO.IntakeIOInputs intakeInputs = new IntakeIO.IntakeIOInputs();
  private final String logRoot;

  private State state = State.IDLE;
  private double targetVoltage = TelemetryUtil.roundToTwoDecimals(kIdleVoltage);
  private BooleanSupplier ignoreLimitsSupplier = () -> false;
  private double lastTargetVoltageDashboardWrite = Double.NaN;

  public Intake(IntakeIO io) {
    this(io, "");
  } // End Intake Constructor

  public Intake(IntakeIO io, String logRoot) {
    intakeIO = io;
    this.logRoot = logRoot;
    SmartDashboard.putNumber(kTargetVoltageKey, TelemetryUtil.roundToTwoDecimals(targetVoltage));
  } // End Intake Constructor

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.recordOutput(logRoot + "Subsystems/Intake/Inputs/MotorConnected", intakeInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Intake/Inputs/AppliedVolts", TelemetryUtil.roundToTwoDecimals(intakeInputs.appliedVolts));
    Logger.recordOutput(logRoot + "Subsystems/Intake/Inputs/SupplyCurrentAmps", TelemetryUtil.roundToTwoDecimals(intakeInputs.supplyCurrentAmps));
    Logger.recordOutput(logRoot + "Subsystems/Intake/TargetVolts", TelemetryUtil.roundToTwoDecimals(getTargetVoltage()));
    Logger.recordOutput(logRoot + "Subsystems/Intake/IsIntaking", state.name() == State.INTAKING.name());
    Logger.recordOutput(logRoot + "Subsystems/Intake/State", state.name());

    if (DriverStation.isDisabled()) {
      intakeIO.stop();
      lastTargetVoltageDashboardWrite = Double.NaN;
      return;
    }

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
    targetVoltage = TelemetryUtil.roundToTwoDecimals(targetVoltage);
    SmartDashboard.putNumber(kTargetVoltageKey, targetVoltage);
    lastTargetVoltageDashboardWrite = targetVoltage;

    // Set the Intake voltage based on the current state.
    switch (state) {
      case IDLE:
        intakeIO.stop();
        break;
      case INTAKING:
      case REVERSING:
      case MANUAL:
        intakeIO.setVoltage(targetVoltage, ignoreLimitsSupplier.getAsBoolean());
        break;
      default:
        intakeIO.stop();
        break;
    }
  } // End periodic

  /** Set state to Idle (motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
    setTargetVoltage(kIdleVoltage);
  } // End setIdleState

  /** Set state to Intaking (pull in at intaking voltage). */
  public void setIntakingState() {
    state = State.INTAKING;
    setTargetVoltage(kIntakingVoltage);
  } // End setIntakingState

  /** Set state to Reversing (spit out at reversing voltage). */
  public void setReversingState() {
    state = State.REVERSING;
    setTargetVoltage(kReversingVoltage);
  } // End setReversingState

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
}
