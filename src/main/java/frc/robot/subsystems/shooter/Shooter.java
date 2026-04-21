package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.State;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.transfer.Transfer;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.AllianceUtil;

/** Coordinates Agitator, Transfer, Turret, Hood, Flywheel; updates targets from lookup; exposes ready-to-shoot. */
public class Shooter extends SubsystemBase {

  private final Drive drive;
  @SuppressWarnings("unused")
  private final Agitator agitator;
  @SuppressWarnings("unused")
  private final Transfer transfer;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final boolean hoodEnabled;

  private BooleanSupplier shootCommandScheduledSupplier = () -> false;
  /** Set by ShootWhenReadyCommand in initialize/end so active is true when running. */
  private volatile boolean shootCommandActive = false;
  /** When true, ShooterCommands.setShooterTarget will not apply calculator to Hood/Flywheel (Manual Override). */
  private BooleanSupplier manualOverrideSupplier = () -> false;

  /** Supplies whether this robot uses red-side field conventions (hub X, zones). */
  private BooleanSupplier isRedAllianceSupplier = AllianceUtil::isRedAlliance;

  /** Prefix for {@link Logger} keys under {@code ShooterCommand/…} and passed to {@link ShooterCommands#setShooterTarget}; empty or a segment ending with {@code '/'}. */
  private final String logRoot;

  /** Will automatically select the shooting target (Hub, left/right passing zones) when true, when false it will target the hub */
  public boolean autoSelectShootingTarget = true;

  /** Accumulated time flywheel has been off target velocity; reset when back on target. */
  private double flywheelOffTargetGraceTimerSec = 0.0;
  private double flywheelGracePrevTimestampSec = Double.NaN;

  public Shooter(
      Drive drive,
      Agitator agitator,
      Transfer transfer,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      boolean hoodEnabled) {
    this(drive, agitator, transfer, turret, hood, flywheel, hoodEnabled, "");
  } // End Shooter Constructor

  public Shooter(
      Drive drive,
      Agitator agitator,
      Transfer transfer,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      boolean hoodEnabled,
      String logRoot) {
    this.drive = drive;
    this.agitator = agitator;
    this.transfer = transfer;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.hoodEnabled = hoodEnabled;
    this.logRoot = logRoot != null ? logRoot : "";
  } // End Shooter Constructor

  /** Set by RobotContainer so Shooter can log whether shoot-when-ready is active. */
  public void setShootCommandScheduledSupplier(BooleanSupplier supplier) {
    shootCommandScheduledSupplier = supplier != null ? supplier : () -> false;
  } // End setShootCommandScheduledSupplier

  /** Set by RobotContainer so calculator does not overwrite Hood/Flywheel when operator is in manual override. */
  public void setManualOverrideSupplier(BooleanSupplier supplier) {
    manualOverrideSupplier = supplier != null ? supplier : () -> false;
  } // End setManualOverrideSupplier

  public void setIsRedAllianceSupplier(BooleanSupplier supplier) {
    isRedAllianceSupplier = supplier != null ? supplier : AllianceUtil::isRedAlliance;
  } // End setIsRedAllianceSupplier

  /** @return {@link #isRedAllianceSupplier} */
  public boolean isOnRedAllianceSide() {
    return isRedAllianceSupplier.getAsBoolean();
  } // End isOnRedAllianceSide

  public void setShootCommandActive(boolean active) {
    shootCommandActive = active;
  } // End setShootCommandActive

  /** @return true if {@link #shootCommandScheduledSupplier} or {@link #shootCommandActive} is true */
  public boolean isShootCommandActive() {
    return shootCommandScheduledSupplier.getAsBoolean() || shootCommandActive;
  } // End isShootCommandActive

  @Override
  public void periodic() {
    double nowSec = Timer.getFPGATimestamp();
    double dtSec =
        Double.isNaN(flywheelGracePrevTimestampSec) ? 0.0 : (nowSec - flywheelGracePrevTimestampSec);
    flywheelGracePrevTimestampSec = nowSec;
    if (flywheel.atTargetVelocity()) {
      flywheelOffTargetGraceTimerSec = 0.0;
    } else {
      flywheelOffTargetGraceTimerSec += dtSec;
    }

    Logger.recordOutput(logRoot + "ShooterCommand/Target", ShooterCommands.getShooterTargetName(drive));
    Logger.recordOutput(logRoot + "ShooterCommand/ShootWhenReadyCommandActive", shootCommandScheduledSupplier.getAsBoolean() || shootCommandActive);
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/IsReadyToShoot", isReadyToShoot());
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/AllianceZoneOk", !ShooterCommands.isShooterTargetHub(drive) || AllianceUtil.isInAllianceZone(drive.getPose().getX(), isRedAllianceSupplier.getAsBoolean()));
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/HubMinDistanceOk", hubAutoshootDistanceOk());
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/TurretTargetInRange", turret.isTargetInRange());
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/TurretAtTarget", turret.atTarget());
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/HoodAtTarget", !hoodEnabled || hood.atTarget());
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/FlywheelAtTarget", flywheel.atTargetVelocity());
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/FlywheelAtTargetWithinGracePeriod", !(flywheelOffTargetGraceTimerSec >= ShooterConstants.kFlywheelOffTargetGraceSec));
    Logger.recordOutput(logRoot + "ShooterCommand/Ready/FlywheelNotIdle", flywheel.getState() != State.IDLE);

    ShooterCommands.setShooterTarget(drive, turret, hood, flywheel, hoodEnabled, !manualOverrideSupplier.getAsBoolean(), logRoot);
  } // End periodic

  /**
   * Turret target in range and on target, Flywheel not Idle and at target speed (with {@link
   * ShooterConstants#kFlywheelOffTargetGraceSec} grace after leaving target speed); (Optional) Hood at target. When
   * shooter target is hub, robot must be in alliance zone and at least {@link
   * ShooterConstants#kMinHubAutoshootDistanceM} from hub center.
   */
  public boolean isReadyToShoot() {
    if (ShooterCommands.isShooterTargetHub(drive)) {
      if (!AllianceUtil.isInAllianceZone(drive.getPose().getX(), isRedAllianceSupplier.getAsBoolean())) {
        return false;
      }
      if (!hubAutoshootDistanceOk()) {
        return false;
      }
    }
    if (!turret.isTargetInRange()) return false;
    if (!turret.atTarget()) return false;
    if (flywheel.getState() == State.IDLE) return false;
    if (flywheelOffTargetGraceTimerSec >= ShooterConstants.kFlywheelOffTargetGraceSec) return false;
    if (hoodEnabled && !hood.atTarget()) return false;
    return true;
  } // End isReadyToShoot

  /** True if not shooting at hub, or robot is far enough from the alliance hub for autoshoot. */
  private boolean hubAutoshootDistanceOk() {
    if (!ShooterCommands.isShooterTargetHub(drive)) {
      return true;
    }
    Translation2d hubCenter = isRedAllianceSupplier.getAsBoolean()
        ? FieldConstants.RED_HUB_CENTER
        : FieldConstants.BLUE_HUB_CENTER;
    return drive.getPose().getTranslation().getDistance(hubCenter) >= ShooterConstants.kMinHubAutoshootDistanceM;
  } // End hubAutoshootDistanceOk
}
