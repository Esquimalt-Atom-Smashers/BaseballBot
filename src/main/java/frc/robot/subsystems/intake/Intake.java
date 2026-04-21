package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem: field-to-storage transfer. */
public class Intake extends SubsystemBase {

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
  private double targetVoltage = kIdleVoltage;
  private BooleanSupplier ignoreLimitsSupplier = () -> false;

  public Intake(IntakeIO io) {
    this(io, "");
  } // End Intake Constructor

  public Intake(IntakeIO io, String logRoot) {
    intakeIO = io;
    this.logRoot = logRoot;
  } // End Intake Constructor

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.recordOutput(logRoot + "Subsystems/Intake/Inputs/MotorConnected", intakeInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Intake/Inputs/AppliedVolts", intakeInputs.appliedVolts);
    Logger.recordOutput(logRoot + "Subsystems/Intake/Inputs/SupplyCurrentAmps", intakeInputs.supplyCurrentAmps);
    Logger.recordOutput(logRoot + "Subsystems/Intake/TargetVolts", getTargetVoltage());
    Logger.recordOutput(logRoot + "Subsystems/Intake/IsIntaking", state.name() == State.INTAKING.name());
    Logger.recordOutput(logRoot + "Subsystems/Intake/State", state.name());

    if (DriverStation.isDisabled()) {
      intakeIO.stop();
      return;
    }

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
    targetVoltage = kIdleVoltage;
  } // End setIdleState

  /** Set state to Intaking (pull in at intaking voltage). */
  public void setIntakingState() {
    state = State.INTAKING;
    targetVoltage = kIntakingVoltage;
  } // End setIntakingState

  /** Set state to Reversing (spit out at reversing voltage). */
  public void setReversingState() {
    state = State.REVERSING;
    targetVoltage = kReversingVoltage;
  } // End setReversingState

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState

  /** Set the target voltage. */
  public void setTargetVoltage(double volts) {
    targetVoltage = volts;
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
    if (getTargetVoltage() == kIdleVoltage) setIdleState();
  } // End stepVoltage
}
