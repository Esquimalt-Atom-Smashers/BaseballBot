package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Hood subsystem: one motor with onboard position control. */
public class Hood extends SubsystemBase {

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

  public Hood(HoodIO io) {
    this(io, "");
  } // End Hood Constructor

  public Hood(HoodIO io, String logRoot) {
    hoodIO = io;
    this.logRoot = logRoot;

    SmartDashboard.putNumber("Hood/kP", kP);
    SmartDashboard.putNumber("Hood/kI", kI);
    SmartDashboard.putNumber("Hood/kD", kD);
    SmartDashboard.putNumber("Hood/TargetPositionDeg", Units.radiansToDegrees(targetAngleRad));
  } // End Hood Constructor

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/MotorConnected", hoodInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/PositionDeg", Units.radiansToDegrees(hoodInputs.positionRads));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/VelocityDegPerSec", Units.radiansToDegrees(hoodInputs.velocityRadsPerSec));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/AppliedVolts", hoodInputs.appliedVolts);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/Inputs/SupplyCurrentAmps", hoodInputs.supplyCurrentAmps);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/TargetPositionAngle", Units.radiansToDegrees(targetAngleRad));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/AtTargetPosition", atTarget());
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Hood/State", state.name());

    if (DriverStation.isDisabled()) {
      state = State.IDLE;
      hoodIO.stop();
      return;
    }

    // Update the target position.
    targetAngleRad = getSetpointRad();

    if (targetAngleRad != lastTargetAngleRad && state != State.MANUAL) {
      setState(State.TRACKING);
    }
    lastTargetAngleRad = targetAngleRad;

    if (!atTarget() && state == State.AT_TARGET) {
      state = State.TRACKING;
    } else if (atTarget() && state == State.TRACKING) {
      state = State.AT_TARGET;
    }

    hoodIO.setTargetPosition(state == State.IDLE ? kDisabledAngleRad : getSetpointRad());
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
    targetAngleRad = ignoreLimitsSupplier.getAsBoolean() ? targetRad : clampTargetAngle(targetRad);
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

  /** Set supplier for ignoring limits. */
  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitsSupplier = supplier != null ? supplier : () -> false;
  } // End setIgnoreLimitsSupplier

  /** Step the target angle in radians. */
  public void stepPositionRad(double stepPositionRad) {
    state = State.MANUAL;
    setTargetAngleRad(getTargetAngleRad() + stepPositionRad);
  } // End stepPositionRad
}
