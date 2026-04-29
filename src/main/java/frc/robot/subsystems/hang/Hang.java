package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TelemetryUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Hang subsystem: one motor with onboard position control. */
public class Hang extends SubsystemBase {
  private static final String kTargetPositionInchesKey = "Hang/TargetPositionInches";
  private static final double kTargetPositionWidgetEpsilonInches = 1e-3;

  /** Hang state: Idle, Stored, Hanging, Level_1, or Manual. */
  public enum State {
    IDLE,
    STORED,
    HANGING,
    LEVEL_1,
    MANUAL
  } // End State enum

  private final HangIO hangIO;
  private final HangIO.HangIOInputs hangInputs = new HangIO.HangIOInputs();
  private final String logRoot;

  private State state = State.IDLE;
  private double targetPositionMeters =
      Units.inchesToMeters(TelemetryUtil.roundToTwoDecimals(Units.metersToInches(kStoredPositionMeters)));
  private BooleanSupplier ignoreLimitsSupplier = () -> false;
  private double lastTargetPositionDashboardWriteInches = Double.NaN;

  public Hang(HangIO io) {
    this(io, "");
  } // End Hang Constructor

  public Hang(HangIO io, String logRoot) {
    hangIO = io;
    this.logRoot = logRoot;

    SmartDashboard.putNumber("Hang/kP", kP);
    SmartDashboard.putNumber("Hang/kI", kI);
    SmartDashboard.putNumber("Hang/kD", kD);
    SmartDashboard.putNumber(kTargetPositionInchesKey, TelemetryUtil.roundToTwoDecimals(Units.metersToInches(targetPositionMeters)));
  } // End Hang Constructor

  @Override
  public void periodic() {
    hangIO.updateInputs(hangInputs);
    Logger.recordOutput(logRoot + "Subsystems/Hang/Inputs/MotorConnected", hangInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Hang/Inputs/PositionInches", TelemetryUtil.roundToTwoDecimals(Units.metersToInches(hangInputs.positionMeters)));
    Logger.recordOutput(logRoot + "Subsystems/Hang/Inputs/VelocityInchesPerSec", Units.metersToInches(hangInputs.velocityMetersPerSec));
    Logger.recordOutput(logRoot + "Subsystems/Hang/Inputs/AppliedVolts", TelemetryUtil.roundToTwoDecimals(hangInputs.appliedVolts));
    Logger.recordOutput(logRoot + "Subsystems/Hang/Inputs/SupplyCurrentAmps", TelemetryUtil.roundToTwoDecimals(hangInputs.supplyCurrentAmps));
    Logger.recordOutput(logRoot + "Subsystems/Hang/TargetPositionInches", TelemetryUtil.roundToTwoDecimals(Units.metersToInches(targetPositionMeters)));
    Logger.recordOutput(logRoot + "Subsystems/Hang/AtTargetPosition", atTargetPosition());
    Logger.recordOutput(logRoot + "Subsystems/Hang/State", state.name());

    if (DriverStation.isDisabled()) {
      hangIO.stop();
      lastTargetPositionDashboardWriteInches = Double.NaN;
      return;
    }

    double targetInches = SmartDashboard.getNumber(kTargetPositionInchesKey,
        TelemetryUtil.roundToTwoDecimals(Units.metersToInches(targetPositionMeters)));
    boolean dashboardTargetEdited = !Double.isNaN(lastTargetPositionDashboardWriteInches)
        && Math.abs(targetInches - lastTargetPositionDashboardWriteInches) > kTargetPositionWidgetEpsilonInches;
    if (dashboardTargetEdited) {
      setTargetPositionMeters(Units.inchesToMeters(targetInches));
      if (state == State.IDLE) {
        state = State.MANUAL;
      }
    }
    double publishedTargetInches = TelemetryUtil.roundToTwoDecimals(Units.metersToInches(targetPositionMeters));
    SmartDashboard.putNumber(kTargetPositionInchesKey, publishedTargetInches);
    lastTargetPositionDashboardWriteInches = publishedTargetInches;

    // Update the target position.
    targetPositionMeters = getSetpointMeters();

    // Set the Hang position based on the current state.
    switch (state) {
      case STORED:
      case HANGING:
      case LEVEL_1:
      case MANUAL:
        hangIO.setTargetPosition(targetPositionMeters);
        break;
      case IDLE:
      default:
        hangIO.stop();
        break;
    }
  } // End periodic

  /** Set state to Idle (motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
    setTargetPositionMeters(getPositionMeters());
    hangIO.stop();
  } // End setIdleState

  /** Set state to Stored (retracted position). */
  public void setStoredState() {
    state = State.STORED;
    setTargetPositionMeters(kStoredPositionMeters);
  } // End setStoredState

  /** Set state to Hanging (retracted part way). */
  public void setHangingState() {
    state = State.HANGING;
    setTargetPositionMeters(kHangingPositionMeters);
  } // End setHangingState

  /** Set state to Level 1 (extended position). */
  public void setLevel1State() {
    state = State.LEVEL_1;
    setTargetPositionMeters(kLevel1PositionMeters);
  } // End setLevel1State

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState


  /** Get the current target position in meters. */
  public double getTargetPositionMeters() {
    return targetPositionMeters;
  } // End getTargetPositionMeters
  
  /** Set target position in meters (clamped to travel limits). */
  public void setTargetPositionMeters(double targetMeters) {
    double clampedMeters = ignoreLimitsSupplier.getAsBoolean() ? targetMeters : clampTargetPosition(targetMeters);
    targetPositionMeters = Units.inchesToMeters(TelemetryUtil.roundToTwoDecimals(Units.metersToInches(clampedMeters)));
  } // End setTargetPositionMeters

  /** Measured position in meters. */
  public double getPositionMeters() {
    return hangInputs.positionMeters;
  } // End getPositionMeters

  /** Whether Hang is at target position within tolerance. */
  public boolean atTargetPosition() {
    return Math.abs(getPositionMeters() - targetPositionMeters) <= kAtTargetToleranceMeters;
  } // End atTargetPosition


  /** Clamp a target extension to mechanical limits. */
  public double clampTargetPosition(double targetMeters) {
    return MathUtil.clamp(targetMeters, kMinMeters, kMaxMeters);
  } // End getClampedTargetPos

  /** Get target position, clamped unless limits override is enabled. */
  private double getSetpointMeters() {
    return ignoreLimitsSupplier.getAsBoolean() ? targetPositionMeters : clampTargetPosition(targetPositionMeters);
  } // End getSetpointMeters


  /** Reset encoder and sync target to max position. */
  public void resetEncoders() {
    hangIO.stop();
    hangIO.resetEncoders();
    setTargetPositionMeters(kStoredPositionMeters);
  } // End resetEncoders

  /** Set supplier for ignoring limits. */
  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitsSupplier = supplier != null ? supplier : () -> false;
  } // End setIgnoreLimitsSupplier
  
  /** Step the target position in meters. */
  public void stepPositionMeters(double stepPositionMeters) {
    state = State.MANUAL;
    setTargetPositionMeters(getTargetPositionMeters() + stepPositionMeters);
  } // End stepPositionMeters
}
