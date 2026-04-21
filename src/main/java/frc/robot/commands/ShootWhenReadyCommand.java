package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.transfer.Transfer;

/**
 * While scheduled: when Shooter is ready (Turret aimed, Flywheel at speed, (Optional) Hood at target),
 * runs Transfer then 0.25 s later Agitator (Shooting). Sets both to Idle on end/cancel.
 */
public class ShootWhenReadyCommand extends Command {

  private static final double kAgitatorDelaySec = 0.25;
  private static final double kLoopPeriodSec = 0.02;

  private final Agitator agitator;
  private final Transfer transfer;
  private final Shooter shooter;
  private final Drive drive;

  private boolean transferOn = false;
  private double timerSec = 0.0;

  private final Supplier<Pose2d> position;

  public ShootWhenReadyCommand(Agitator agitator, Transfer transfer, Shooter shooter, Drive drive, Supplier<Pose2d> position) {
    this.agitator = agitator;
    this.transfer = transfer;
    this.shooter = shooter;
    this.drive = drive;
    this.position = position;
    addRequirements(agitator, transfer, shooter);
  } // End ShootWhenReadyCommand Constructor

  @Override
  public void initialize() {
    transferOn = false;
    timerSec = 0.0;
    shooter.setShootCommandActive(true);
  } // End initialize

  @Override
  public void execute() {
    // Aim at hub if in alliance zone, otherwise depending on the side of the field pass balls back into the alliance zone
    if (shooter.autoSelectShootingTarget) {
      automaticallySelectShootingTarget();
    } else {
      ShooterCommands.clearShooterTargetOverride(drive);
    }

    if (!shooter.isReadyToShoot()) {
      // Reset the states of the Agitator and Transfer if we become "unready" to shoot after the command has started
      agitator.setIdleState();
      transfer.setIdleState();
      transferOn = false;
      return;
    }

    if (!transferOn) {
      transfer.setShootingState();
      transferOn = true;
      timerSec = 0.0;
    }

    timerSec += kLoopPeriodSec;
    if (timerSec >= kAgitatorDelaySec) {
      agitator.setShootingState();
    }
  } // End execute

  /** Automatically selects the shooters target depending on the robots x and y */
  private void automaticallySelectShootingTarget() {
    Pose2d pose = position.get();
    if (isInAllianceZoneWithTolerance(pose.getX())) {
      ShooterCommands.clearShooterTargetOverride(drive);
    } else {
      boolean aboveCenterY = pose.getY() > FieldConstants.FIELD_CENTER_Y_M;
      // Passing spots are alliance-relative; red LEFT/RIGHT are Y-mirrored vs blue.
      boolean passLeft = aboveCenterY ^ shooter.isOnRedAllianceSide();
      if (passLeft) {
        ShooterCommands.setPassingSpotLeft(drive);
      } else {
        ShooterCommands.setPassingSpotRight(drive);
      }
    }
  }

  /** Same as alliance zone check with auto-select tolerance toward field center. */
  private boolean isInAllianceZoneWithTolerance(double robotXM) {
    double tol = ShooterConstants.kAutoSelectShootingTargetAllianceZoneTolerance;
    return shooter.isOnRedAllianceSide()
        ? robotXM > FieldConstants.FIELD_LENGTH_M - FieldConstants.ALLIANCE_ZONE_M - tol
        : robotXM < FieldConstants.ALLIANCE_ZONE_M + tol;
  } // End isInAllianceZoneWithTolerance

  @Override
  public void end(boolean interrupted) {
    shooter.setShootCommandActive(false);
    transfer.setIdleState();
    agitator.setIdleState();
  } // End end

  @Override
  public boolean isFinished() {
    return false; // runs until cancelled (toggle off)
  } // End isFinished
}
