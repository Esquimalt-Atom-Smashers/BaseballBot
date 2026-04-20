package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kIdleVelocityRadPerSec;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
    POSITIONING,
    AT_POSITION,
    MANUAL
  } // End State enum

  private final HoodIO hoodIO;
  private final HoodIO.HoodIOInputs hoodInputs = new HoodIO.HoodIOInputs();

  private State state = State.IDLE;
  private double targetAngleRad = kDisabledAngleRad;
  private double lastTargetAngleRad = kDisabledAngleRad;
  private BooleanSupplier ignoreLimitsSupplier = () -> false;

  public Hood(HoodIO io) {
    hoodIO = io;

    SmartDashboard.putNumber("Hood/kP", kP);
    SmartDashboard.putNumber("Hood/kI", kI);
    SmartDashboard.putNumber("Hood/kD", kD);
    SmartDashboard.putNumber("Hood/TargetPositionDeg", Units.radiansToDegrees(targetAngleRad));
  } // End Hood Constructor

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/MotorConnected", hoodInputs.motorConnected);
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/PositionDeg", Units.radiansToDegrees(hoodInputs.positionRads));
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/VelocityDegPerSec", Units.radiansToDegrees(hoodInputs.velocityRadsPerSec));
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/AppliedVolts", hoodInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Shooter/Hood/Inputs/SupplyCurrentAmps", hoodInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Shooter/Hood/TargetPositionAngle", Units.radiansToDegrees(targetAngleRad));
    Logger.recordOutput("Subsystems/Shooter/Hood/AtTargetPosition", atTarget());
    Logger.recordOutput("Subsystems/Shooter/Hood/State", state.name());

    if (DriverStation.isDisabled()) {
      state = State.IDLE;
      hoodIO.stop();
      return;
    }

    if (targetAngleRad != lastTargetAngleRad) {
      setState(State.POSITIONING);
    }

    lastTargetAngleRad = targetAngleRad;

    if (!atTarget() && state == State.AT_POSITION) {
      state = State.POSITIONING;
    }

    if (atTarget() && state == State.POSITIONING) {
      state = State.AT_POSITION;
    }

    hoodIO.setTargetPosition(state == State.IDLE ? kDisabledAngleRad : getTargetAngleRad());

    state = ignoreLimitsSupplier.getAsBoolean() ? State.MANUAL : (atTarget() ? State.AT_POSITION : State.POSITIONING);

    // Set the Hood target position based on the current state.
    double targetPositionRad = getSetpointRad();
    hoodIO.setTargetPosition(targetPositionRad);
  } // End periodic

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState

  /** Set the Hood state. */
  public void setState(State newState) {
    state = newState;
  } // End setState

  /** Get the current Hood angle. */
  public double getAngleRad() {
    return hoodInputs.positionRads;
  } // End getAngleRad

  /** Get the current target angle. */
  public double getTargetAngleRad() {
    return targetAngleRad;
  } // End getTargetAngleRad

  /** Set the target angle. Clamped to min/max in periodic. */
  public void setTargetAngleRad(double targetRad) {
    targetAngleRad = clampTargetAngle(targetRad);
  } // End setTargetAngleRad

  /** Whether the Hood is at the target angle within tolerance. */
  public boolean atTarget() {
    return Math.abs(getAngleRad() - getSetpointRad()) <= kAtTargetToleranceRad;
  } // End atTarget


  /** Clamp a target angle to mechanical limits. */
  public double clampTargetAngle(double targetRad) {
    return MathUtil.clamp(targetRad, kMinAngleRad, kMaxAngleRad);
  } // End clampTargetAngle

  /** Get target angle, clamped to hood travel limits. */
  private double getSetpointRad() {
    return ignoreLimitsSupplier.getAsBoolean() ? targetAngleRad : clampTargetAngle(targetAngleRad);
  } // End getSetpointRad


  /** Set supplier for ignoring limits. */
  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitsSupplier = supplier != null ? supplier : () -> false;
  } // End setIgnoreLimitsSupplier

  /** Step the target position in Turret frame. */
  public void stepPositionRad(double stepPositionRad) {
    setTargetAngleRad(getAngleRad() + stepPositionRad);
    state = State.MANUAL;
  } // End stepPositionRad
}
