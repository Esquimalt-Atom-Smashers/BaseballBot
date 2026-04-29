package frc.robot.subsystems.extender;

import static frc.robot.subsystems.extender.ExtenderConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TelemetryUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Extender subsystem: one motor with onboard position control. */
public class Extender extends SubsystemBase {
  private static final String kTargetPositionDegKey = "Extender/TargetPositionDeg";
  private static final double kTargetPositionWidgetEpsilonDeg = 1e-3;

  /** Extender state: Idle, Retracted (facing up), Partial, Extended (facing forward) or Manual. */
  public enum State {
    IDLE,
    RETRACTED,
    PARTIAL,
    EXTENDED,
    MANUAL
  } // End State enum

  private final ExtenderIO extenderIO;
  private final ExtenderIO.ExtenderIOInputs extenderInputs = new ExtenderIO.ExtenderIOInputs();
  private final String logRoot;

  private State state = State.RETRACTED;
  private double targetPositionRad =
      Units.degreesToRadians(TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(kUpExtenderRad)));
  private BooleanSupplier ignoreLimitsSupplier = () -> false;
  private double lastTargetPositionDashboardWriteDeg = Double.NaN;

  public Extender(ExtenderIO io) {
    this(io, "");
  } // End Extender Constructor

  public Extender(ExtenderIO io, String logRoot) {
    extenderIO = io;
    this.logRoot = logRoot;

    SmartDashboard.putNumber("Extender/kP", kP);
    SmartDashboard.putNumber("Extender/kI", kI);
    SmartDashboard.putNumber("Extender/kD", kD);
    SmartDashboard.putNumber(kTargetPositionDegKey, TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(targetPositionRad)));
  } // End Extender Constructor

  @Override
  public void periodic() {
    extenderIO.updateInputs(extenderInputs);
    Logger.recordOutput(logRoot + "Subsystems/Extender/Inputs/MotorConnected", extenderInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Extender/Inputs/PositionDeg", TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(extenderInputs.positionRads)));
    Logger.recordOutput(logRoot + "Subsystems/Extender/Inputs/VelocityDegPerSec", Units.radiansToDegrees(extenderInputs.velocityRadsPerSec));
    Logger.recordOutput(logRoot + "Subsystems/Extender/Inputs/AppliedVolts", TelemetryUtil.roundToTwoDecimals(extenderInputs.appliedVolts));
    Logger.recordOutput(logRoot + "Subsystems/Extender/Inputs/SupplyCurrentAmps", TelemetryUtil.roundToTwoDecimals(extenderInputs.supplyCurrentAmps));
    Logger.recordOutput(logRoot + "Subsystems/Extender/TargetPositionDeg", TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(targetPositionRad)));
    Logger.recordOutput(logRoot + "Subsystems/Extender/AtTargetPosition", atTargetPosition());
    Logger.recordOutput(logRoot + "Subsystems/Extender/State", state.name());

    if (DriverStation.isDisabled()) {
      extenderIO.stop();
      lastTargetPositionDashboardWriteDeg = Double.NaN;
      return;
    }

    double targetDeg = SmartDashboard.getNumber(kTargetPositionDegKey,
        TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(targetPositionRad)));
    boolean dashboardTargetEdited = !Double.isNaN(lastTargetPositionDashboardWriteDeg)
        && Math.abs(targetDeg - lastTargetPositionDashboardWriteDeg) > kTargetPositionWidgetEpsilonDeg;
    if (dashboardTargetEdited) {
      setTargetPositionRad(Units.degreesToRadians(targetDeg));
      if (state == State.IDLE) {
        state = State.MANUAL;
      }
    }
    double publishedTargetDeg = TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(targetPositionRad));
    SmartDashboard.putNumber(kTargetPositionDegKey, publishedTargetDeg);
    lastTargetPositionDashboardWriteDeg = publishedTargetDeg;

    // Update the target position.
    targetPositionRad = getSetpointRad();

    // Set the Extender position based on the current state.
    switch (state) {
      case RETRACTED:
      case PARTIAL:
        extenderIO.setTargetPosition(getSetpointRad());
        break;
      case EXTENDED:
        if (getPositionRad() > targetPositionRad - kAtTargetToleranceRad) {
          extenderIO.stop();
        } else {
          extenderIO.setTargetPosition(targetPositionRad);
        }
        break;
      case MANUAL:
        extenderIO.setTargetPosition(targetPositionRad);
        break;
      case IDLE:
        if (getPositionRad() > kExtendedExtenderRad - kAtTargetToleranceRad) {
          setExtendedState();
          break;
        }
      default:
        extenderIO.stop();
        break;
    }
  } // End periodic

  /** Set state to Idle (motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
    setTargetPositionRad(getPositionRad());
    extenderIO.stop();
  } // End setIdleState

  /** Set state to Retracted (up position). */
  public void setRetractedState() {
    state = State.RETRACTED;
    setTargetPositionRad(kUpExtenderRad);
  } // End setRetractedState

  /** Set state to Partial (middle position). */
  public void setPartialState() {
    state = State.PARTIAL;
    setTargetPositionRad(kPartialExtenderRad);
  } // End setPartialState

  /** Set state to Extended (down; rest on bumpers when there). */
  public void setExtendedState() {
    state = State.EXTENDED;
    setTargetPositionRad(kExtendedExtenderRad);
  } // End setExtendedState

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState


  /** Measured position in radians. */
  public double getPositionRad() {
    return extenderInputs.positionRads;
  } // End getPositionRad  

  /** Get the current target position in radians. */
  public double getTargetPositionRad() {
    return targetPositionRad;
  } // End getTargetPositionRad

  /** Set target position in radians (clamped to travel limits if limits override is not enabled). */
  public void setTargetPositionRad(double targetRad) {
    double clampedRad = ignoreLimitsSupplier.getAsBoolean() ? targetRad : clampTargetPosition(targetRad);
    targetPositionRad = Units.degreesToRadians(TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(clampedRad)));
  } // End setTargetPositionRad

  /** Whether Extender is at target position within tolerance. */
  public boolean atTargetPosition() {
    return Math.abs(getPositionRad() - targetPositionRad) <= kAtTargetToleranceRad;
  } // End atTargetPosition

  
  /** Clamp a target angle to mechanical limits. */
  public double clampTargetPosition(double targetRad) {
    return MathUtil.clamp(targetRad, kMinRad, kMaxRad);
  } // End getClampedTargetPos

  /** Get target position, clamped unless limits override is enabled. */
  private double getSetpointRad() {
    return ignoreLimitsSupplier.getAsBoolean() ? targetPositionRad : clampTargetPosition(targetPositionRad);
  } // End getSetpointRad


  /** Stop the motor and reset the encoder to the maximum position. */
  public void resetEncoders() {
    extenderIO.stop();
    extenderIO.resetEncoders();
    setTargetPositionRad(kMaxRad);
  } // End resetEncoders

  /** Set supplier for ignoring limits. */
  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitsSupplier = supplier != null ? supplier : () -> false;
  } // End setIgnoreLimitsSupplier
  
  /** Step the target position in radians. */
  public void stepPositionRad(double stepPositionRad) {
    state = State.MANUAL;
    setTargetPositionRad(getTargetPositionRad() + stepPositionRad);
  } // End stepPositionRad
}
