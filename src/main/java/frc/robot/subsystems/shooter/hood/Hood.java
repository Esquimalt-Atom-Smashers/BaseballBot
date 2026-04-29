package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TelemetryUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Hood subsystem: one motor with onboard position control. */
public class Hood extends SubsystemBase {

  private static final String kTargetPositionDegKey = "Hood/TargetPositionDeg";
  private static final String kTargetSetPosKey = "Hood/TargetServoSetPos";

  /** Minimum change on the raw servo (0–1) NT widget to count as operator input vs our publish echo. */
  private static final double kServoSetWidgetEpsilon = 1e-3;

  /** Hood state: Idle, Tracking (approaching target), At_Target, or Manual. */
  public enum State {
    IDLE,
    TRACKING,
    AT_TARGET,
    MANUAL
  } // End State enum

  private final HoodIO hoodIO;
  private final HoodIO.HoodIOInputs hoodInputs = new HoodIO.HoodIOInputs();
  private final String logRoot;

  private State state = State.IDLE;
  private double targetAngleRad = kDisabledAngleRad;
  private double lastTargetAngleRad = kDisabledAngleRad;
  private BooleanSupplier ignoreLimitsSupplier = () -> false;

  private BooleanSupplier useSmartDashboardSupplier = () -> false;
  private double lastHoodTargetDashboardWriteDeg = Double.NaN;
  private double lastHoodTargetServoSetWrite = Double.NaN;

  public Hood(HoodIO io) {
    this(io, "");
  } // End Hood Constructor

  public Hood(HoodIO io, String logRoot) {
    hoodIO = io;
    this.logRoot = logRoot;

    SmartDashboard.putNumber("Hood/kP", kP);
    SmartDashboard.putNumber("Hood/kI", kI);
    SmartDashboard.putNumber("Hood/kD", kD);
    SmartDashboard.putNumber(kTargetPositionDegKey, TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(targetAngleRad)));
    SmartDashboard.putNumber(kTargetSetPosKey, 0.5);
  } // End Hood Constructor

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);

    if (DriverStation.isDisabled()) {
      state = State.IDLE;
      hoodIO.stop();
      lastHoodTargetDashboardWriteDeg = Double.NaN;
      lastHoodTargetServoSetWrite = Double.NaN;
      return;
    }

    // Manual override: SmartDashboard and/or stepPositionRad (MANUAL). Do not read NT every MANUAL cycle — that would
    // undo steps; resume NT control when the widget differs from our last publish value.
    if (useSmartDashboardSupplier.getAsBoolean()) {
      double setpointDeg = Units.radiansToDegrees(getSetpointRad());
      double deg = SmartDashboard.getNumber(kTargetPositionDegKey, setpointDeg);
      double degRounded = TelemetryUtil.roundToTwoDecimals(deg);
      if (state == State.MANUAL) {
        if (!Double.isNaN(lastHoodTargetDashboardWriteDeg)
            && Math.abs(deg - lastHoodTargetDashboardWriteDeg) > Units.radiansToDegrees(kAtTargetToleranceRad)) {
          setTargetAngleRad(Units.degreesToRadians(degRounded));
          setState(State.TRACKING);
        } else {
          setTargetAngleRad(getSetpointRad());
        }
      } else {
        setTargetAngleRad(Units.degreesToRadians(degRounded));
      }
    } else {
      setTargetAngleRad(getSetpointRad());
    }
    double publishedDeg = Units.radiansToDegrees(getSetpointRad());
    double roundedPublishedDeg = TelemetryUtil.roundToTwoDecimals(publishedDeg);
    SmartDashboard.putNumber(kTargetPositionDegKey, roundedPublishedDeg);
    lastHoodTargetDashboardWriteDeg = roundedPublishedDeg;

    boolean manualDirectServo = false;
    double manualServoSetClamped = 0.5;
    if (useSmartDashboardSupplier.getAsBoolean() && state == State.MANUAL) {
      double defaultServo = servoSetForDashboardRad(getSetpointRad());
      double rawServo = SmartDashboard.getNumber(kTargetSetPosKey, defaultServo);
      if (!Double.isNaN(lastHoodTargetServoSetWrite)
          && Math.abs(rawServo - lastHoodTargetServoSetWrite) > kServoSetWidgetEpsilon) {
        manualDirectServo = true;
        manualServoSetClamped = MathUtil.clamp(rawServo, 0.0, 1.0);
      }
    }

    if (targetAngleRad != lastTargetAngleRad && state != State.MANUAL) {
      setState(State.TRACKING);
    }
    lastTargetAngleRad = targetAngleRad;

    switch (state) {
      case TRACKING:
      case AT_TARGET:
        if (atTarget()) {
          setState(State.AT_TARGET);

          // If commanded to disabled angle and measured angle reached that region, transition to IDLE.
          if (Math.abs(targetAngleRad - kDisabledAngleRad) <= kAtTargetToleranceRad
              && (getAngleRad() > kDisabledAngleRad - kAtTargetToleranceRad)) {
            setState(State.IDLE);
          } else {
            hoodIO.setTargetPosition(getSetpointRad());
          }
        } else {
          setState(State.TRACKING);
          hoodIO.setTargetPosition(getSetpointRad());
        }
        break;
      case MANUAL:
        if (manualDirectServo) {
          hoodIO.setServoPosition(manualServoSetClamped);
        } else {
          hoodIO.setTargetPosition(getSetpointRad());
        }
        break;
      case IDLE:
        hoodIO.stop();
        break;
      default:
        hoodIO.stop();
        break;
    }

    double publishedServoSet = manualDirectServo ? manualServoSetClamped : servoSetForDashboardRad(getSetpointRad());
    SmartDashboard.putNumber(kTargetSetPosKey, publishedServoSet);
    lastHoodTargetServoSetWrite = publishedServoSet;

    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/MotorConnected", hoodInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/PositionDeg", TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(hoodInputs.positionRads)));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/VelocityDegPerSec", Units.radiansToDegrees(hoodInputs.velocityRadsPerSec));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/AppliedVolts", TelemetryUtil.roundToTwoDecimals(hoodInputs.appliedVolts));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/AnalogVolts", hoodInputs.analogVolts);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/SupplyCurrentAmps", TelemetryUtil.roundToTwoDecimals(hoodInputs.supplyCurrentAmps));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/TargetPositionAngle", TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(getSetpointRad())));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/UseSmartDashboardWhenManualOverride", useSmartDashboardSupplier.getAsBoolean());
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/AtTargetPosition", atTarget());
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/State", state.name());
  } // End periodic

  /** Set the Hood state. */
  public void setState(State newState) {
    state = newState;
  } // End setState

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState

  /** Get the current Hood angle. */
  public double getAngleRad() {
    return hoodInputs.positionRads;
  } // End getAngleRad

  /** Current target elevation from horizontal (radians). */
  public double getTargetAngleRad() {
    return targetAngleRad;
  } // End getTargetAngleRad

  /**
   * Set target elevation (rad from horizontal). Clamped to travel limits unless {@link #setIgnoreLimitsSupplier}
   * is true.
   */
  public void setTargetAngleRad(double targetRad) {
    double clampedRad = ignoreLimitsSupplier.getAsBoolean() ? targetRad : clampTargetAngle(targetRad);
    targetAngleRad = Units.degreesToRadians(TelemetryUtil.roundToTwoDecimals(Units.radiansToDegrees(clampedRad)));
  } // End setTargetAngleRad

  /** Whether the Hood is at the target angle within tolerance. */
  public boolean atTarget() {
    return Math.abs(getAngleRad() - targetAngleRad) <= kAtTargetToleranceRad;
  } // End atTarget


  /** Clamp a target angle to mechanical limits. */
  public double clampTargetAngle(double targetRad) {
    return MathUtil.clamp(targetRad, kMinAngleRad, kMaxAngleRad);
  } // End clampTargetAngle

  /** Get target angle, clamped to hood travel limits after applying limits when override is off. */
  private double getSetpointRad() {
    return ignoreLimitsSupplier.getAsBoolean() ? targetAngleRad : clampTargetAngle(targetAngleRad);
  } // End getSetpointRad

  /** Servo.set (0–1) from hood angle. */
  private static double servoSetForDashboardRad(double hoodAngleRad) {
    double u = kServoSetAt0deg + kServoSetPerHoodAngleRad * hoodAngleRad;
    return MathUtil.clamp(u, 0.0, 1.0);
  } // End servoSetForDashboardRad

  /** Set supplier for ignoring limits. */
  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitsSupplier = supplier != null ? supplier : () -> false;
  } // End setIgnoreLimitsSupplier

  /** When supplier is true, hood follows SmartDashboard / steps during operator manual override. */
  public void setUseSmartDashboardTarget(BooleanSupplier supplier) {
    useSmartDashboardSupplier = supplier != null ? supplier : () -> false;
  } // End setUseSmartDashboardTarget

  /** Step the target angle in radians. */
  public void stepPositionRad(double stepPositionRad) {
    state = State.MANUAL;
    setTargetAngleRad(getTargetAngleRad() + stepPositionRad);
  } // End stepPositionRad
}
