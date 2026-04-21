package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Turret subsystem: one motor with onboard position control, aimed at a target angle. */
public class Turret extends SubsystemBase {

  /** Turret state: Idle, Tracking (approaching target), At_Target, or Manual. */
  public enum State {
    IDLE,
    TRACKING,
    AT_TARGET,
    MANUAL
  } // End State enum

  private final TurretIO turretIO;
  private final TurretIO.TurretIOInputs turretInputs = new TurretIO.TurretIOInputs();
  private final String logRoot;

  private State state = State.IDLE;

  private BooleanSupplier manualOverrideSupplier = () -> false;
  private BooleanSupplier aimAtTargetSupplier = () -> false;
  private Drive drive;

  private Rotation2d targetRelativeToRobot = kDefaultAimDirectionRobotFrame; // Target in robot frame (0 = forward).
  private double lastTargetPositionRad = 0.0; // Last position setpoint sent to IO.
  private double velocityFeedforwardRadPerSec = 0.0;
  private double lastSmartDashboardTargetPosRad = kDefaultAimDirectionRobotFrame.getRadians();
  private boolean lastManualOverride = false;

  public Turret(TurretIO io) {
    this(io, "");
  } // End Turret Constructor

  public Turret(TurretIO io, String logRoot) {
    turretIO = io;
    this.logRoot = logRoot;

    SmartDashboard.putNumber("Turret/kP", kP);
    SmartDashboard.putNumber("Turret/kI", kI);
    SmartDashboard.putNumber("Turret/kD", kD);
    SmartDashboard.putNumber("Turret/TargetPositionDeg", Units.radiansToDegrees(kDefaultAimDirectionRobotFrame.getRadians()));
  } // End Turret Constructor

  @Override
  public void periodic() {
    turretIO.updateInputs(turretInputs);

    double targetPositionRad;

    // Disabled: Set target position to current position with no velocity feedforward (holds current position).
    if (DriverStation.isDisabled()) {
      state = State.IDLE;
      velocityFeedforwardRadPerSec = 0.0;
      targetPositionRad = MathUtil.clamp(turretInputs.positionRads, kMinAngleRad, kMaxAngleRad);
    }
    // Enabled: Aim at the target only if aim-at-target is enabled (e.g. ShootWhenReadyCommand active); otherwise hold position.
    // Gets robot relative omega for velocity feedforward for spin compensation (counteracts robot rotation).
    else if (!manualOverrideSupplier.getAsBoolean()) {
      if (aimAtTargetSupplier.getAsBoolean()) {
        setTargetRelativeToRobot(ShooterCommands.getTurretAngleFromShot(drive));
        setVelocityFeedforwardRadPerSec(-drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);
        targetPositionRad = getSetpointRad();
        state = atTarget() ? State.AT_TARGET : State.TRACKING;
      } else {
        state = State.IDLE;
        setVelocityFeedforwardRadPerSec(0.0);
        targetPositionRad = MathUtil.clamp(turretInputs.positionRads, kMinAngleRad, kMaxAngleRad);
      }
    }
    // Manual Override: Manually overriding the Turret position.
    else {
      double targetRobotFrameRad = Units.degreesToRadians(SmartDashboard.getNumber("Turret/TargetPositionDeg", 
          Units.radiansToDegrees(kDefaultAimDirectionRobotFrame.getRadians())));
      boolean manualRisingEdge = !lastManualOverride;
      if (manualRisingEdge) {
        setTargetRelativeToRobot(Rotation2d.fromRadians(getRobotFramePosition().getRadians()));
      } else if (targetRobotFrameRad != lastSmartDashboardTargetPosRad) {
        setTargetRelativeToRobot(Rotation2d.fromRadians(targetRobotFrameRad));
      }
      state = State.MANUAL;

      lastSmartDashboardTargetPosRad = targetRobotFrameRad;
      velocityFeedforwardRadPerSec = 0.0;
      targetPositionRad = getSetpointRad();
    }

    lastTargetPositionRad = targetPositionRad;
    lastManualOverride = manualOverrideSupplier.getAsBoolean();

    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/Inputs/MotorConnected", turretInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/Inputs/PositionDeg", Units.radiansToDegrees(turretInputs.positionRads));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/Inputs/VelocityDegPerSec", Units.radiansToDegrees(turretInputs.velocityRadsPerSec));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/Inputs/AppliedVolts", turretInputs.appliedVolts);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/Inputs/SupplyCurrentAmps", turretInputs.supplyCurrentAmps);
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/TargetPositionDeg", Units.radiansToDegrees(targetPositionRad));
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/TargetRobotFrameDeg", getTargetRelativeToRobot().getDegrees());
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/RobotFrameDeg", getRobotFramePosition().getDegrees());
    Logger.recordOutput(logRoot + "Subsystems/Shooter/Turret/State", state.name());

    turretIO.setTargetPosition(targetPositionRad, velocityFeedforwardRadPerSec);
  } // End periodic

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState

  
  /** Set velocity feedforward. */
  public void setVelocityFeedforwardRadPerSec(double radPerSec) {
    velocityFeedforwardRadPerSec = radPerSec;
  } // End setVelocityFeedforwardRadPerSec

  /** Set manualOverrideSupplier. */
  public void setManualOverrideSupplier(BooleanSupplier supplier) {
    manualOverrideSupplier = supplier != null ? supplier : () -> false;
  } // End setManualOverrideSupplier

  /** Set aimAtTargetSupplier. */
  public void setAimAtTargetSupplier(BooleanSupplier supplier) {
    aimAtTargetSupplier = supplier != null ? supplier : () -> false;
  } // End setAimAtTargetSupplier

  /** Set drive. */
  public void setDrive(Drive drive) {
    this.drive = drive;
  } // End setDrive


  /** Get the current Turret position (robot frame: 0 = forward). */
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(turretInputs.positionRads);
  } // End getPosition

  /** Get the current target position in Turret frame. */
  public Rotation2d getTargetPosition() {
    return Rotation2d.fromRadians(lastTargetPositionRad);
  } // End getTargetPosition

  /** Get the current Turret position in robot frame. 0 = forward, 180° = back. */
  public Rotation2d getRobotFramePosition() {
    return Rotation2d.fromRadians(turretToRobotFrameRad(turretInputs.positionRads));
  } // End getRobotFramePosition

  /** Get the current target position in robot frame. */
  public Rotation2d getRobotFrameTargetPosition() {
    return Rotation2d.fromRadians(turretToRobotFrameRad(lastTargetPositionRad));
  } // End getRobotFrameTargetPosition

  /** Set target in robot frame (0 = forward). */
  public void setTargetRelativeToRobot(Rotation2d angle) {
    targetRelativeToRobot = angle;
  } // End setTargetRelativeToRobot

  /** Get target in robot frame (from shot solver, manual dashboard, or default). */
  public Rotation2d getTargetRelativeToRobot() {
    return targetRelativeToRobot;
  } // End getTargetRelativeToRobot


  /**
   * Convert robot-frame angle (0 = forward, per {@link TurretConstants#kDefaultAimDirectionRobotFrame})
   * to Turret internal frame.
   *
   * This keeps the turret math consistent if you change which robot direction you consider
   * "turret frame 0".
   */
  private static double robotToTurretFrameRad(double robotFrameRad) {
    double turretFrameRad = robotFrameRad - kDefaultAimDirectionRobotFrame.getRadians();

    double twoPi = 2.0 * Math.PI;
    return MathUtil.inputModulus(turretFrameRad, kMinAngleRad, kMinAngleRad + twoPi);
  } // End robotToTurretFrameRad

  /**
   * Convert Turret internal frame (0 = turret internal 0) to robot-frame angle, using
   * {@link TurretConstants#kDefaultAimDirectionRobotFrame} as the offset.
   */
  private static double turretToRobotFrameRad(double turretFrameRad) {
    return turretFrameRad + kDefaultAimDirectionRobotFrame.getRadians();
  } // End turretToRobotFrameRad

  /** Get and convert target (robot frame) to Turret frame and clamp to mechanical limits if manual override is not enabled. */
  private double getSetpointRad() {
    // Convert robot-frame target into turret internal frame (0 = turret internal 0).
    // IMPORTANT: the turret has >180° travel, so we must choose the closest equivalent
    // representation (angle, angle±2π) within limits to avoid taking the long way around.
    double turretRawRad =
        targetRelativeToRobot.getRadians() - kDefaultAimDirectionRobotFrame.getRadians();
    double twoPi = 2.0 * Math.PI;

    double[] candidates = {turretRawRad, turretRawRad + twoPi, turretRawRad - twoPi};

    double best = Double.NaN;
    double bestAbsError = Double.POSITIVE_INFINITY;

    for (double c : candidates) {
      if (c < kMinAngleRad || c > kMaxAngleRad) continue;
      double absError = Math.abs(c - turretInputs.positionRads);
      if (absError < bestAbsError) {
        bestAbsError = absError;
        best = c;
      }
    }

    if (manualOverrideSupplier.getAsBoolean()) {
      // Manual override: still pick the closest *reachable* representation, but do not force
      // a hard clamp if none are in-range (driver may be intentionally commanding past limits).
      return Double.isNaN(best) ? turretRawRad : best;
    }

    if (!Double.isNaN(best)) return best;

    // Normal mode: if the target orientation is unreachable (in the "missing" window),
    // drive to whichever mechanical limit is closer to the *desired target orientation*.
    // (This creates the expected midpoint behavior, e.g. around 140° for -200°..+120°.)
    //
    // Then use current position only as a tie-breaker.
    double toMin = Math.min(
        Math.abs(turretRawRad - kMinAngleRad),
        Math.min(Math.abs((turretRawRad + twoPi) - kMinAngleRad), Math.abs((turretRawRad - twoPi) - kMinAngleRad)));
    double toMax = Math.min(
        Math.abs(turretRawRad - kMaxAngleRad),
        Math.min(Math.abs((turretRawRad + twoPi) - kMaxAngleRad), Math.abs((turretRawRad - twoPi) - kMaxAngleRad)));

    if (toMin < toMax) return kMinAngleRad;
    if (toMax < toMin) return kMaxAngleRad;

    double curToMin = Math.abs(kMinAngleRad - turretInputs.positionRads);
    double curToMax = Math.abs(kMaxAngleRad - turretInputs.positionRads);
    return (curToMin <= curToMax) ? kMinAngleRad : kMaxAngleRad;
  } // End getSetpointRad


  /** Whether the target is reachable within Turret travel limits. */
  public boolean isTargetInRange() {
    double turretRawRad = targetRelativeToRobot.getRadians() - kDefaultAimDirectionRobotFrame.getRadians();
    double twoPi = 2.0 * Math.PI;

    return (turretRawRad >= kMinAngleRad && turretRawRad <= kMaxAngleRad)
        || (turretRawRad + twoPi >= kMinAngleRad && turretRawRad + twoPi <= kMaxAngleRad)
        || (turretRawRad - twoPi >= kMinAngleRad && turretRawRad - twoPi <= kMaxAngleRad);
  } // End isTargetInRange

  /** Whether Turret is at target position within tolerance. */
  public boolean atTarget() {
    double targetTurretRad = getSetpointRad();
    double tol = ShooterCommands.isShooterTargetHub(drive) ? kAtTargetToleranceRad : kAtTargetToleranceNonHubRad;
    return Math.abs(turretInputs.positionRads - targetTurretRad) <= tol;
  } // End atTarget

  
  /** Resets the motors position to 0. */
  public void resetMotorEncoder() {
    turretIO.stop();
    turretIO.resetEncoder();
    setTargetRelativeToRobot(kDefaultAimDirectionRobotFrame);
    lastTargetPositionRad = robotToTurretFrameRad(kDefaultAimDirectionRobotFrame.getRadians());
    SmartDashboard.putNumber("Turret/TargetPositionDeg", Units.radiansToDegrees(kDefaultAimDirectionRobotFrame.getRadians()));
  } // End resetMotorEncoder

  /** Step the target position in Turret frame. */
  public void stepPositionRad(double stepPositionRad) {
    double turretTargetRad = manualOverrideSupplier.getAsBoolean() 
        ? lastTargetPositionRad + stepPositionRad
        : MathUtil.clamp(lastTargetPositionRad + stepPositionRad, kMinAngleRad, kMaxAngleRad);
    setTargetRelativeToRobot(Rotation2d.fromRadians(turretToRobotFrameRad(turretTargetRad)));
  } // End stepPositionRad
}
