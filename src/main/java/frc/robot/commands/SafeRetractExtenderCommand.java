package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.extender.Extender;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.turret.Turret;

/**
 * Cancels auto-shoot, idles flywheel, homes turret to 0°, then retracts extender after a short delay.
 */
public final class SafeRetractExtenderCommand {

  private SafeRetractExtenderCommand() {}

  /**
   * @param driverTurretOverride set true while forcing turret to dashboard 0° target; cleared when done
   */
  public static Command create(
      ShootWhenReadyCommand shootWhenReady,
      Flywheel flywheel,
      Extender extender,
      Intake intake,
      Turret turret,
      Consumer<Boolean> driverTurretOverride) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              if (shootWhenReady.isScheduled()) {
                CommandScheduler.getInstance().cancel(shootWhenReady);
              }
              if (flywheel != null) flywheel.setState(Flywheel.State.IDLE);
              driverTurretOverride.accept(true);
              SmartDashboard.putNumber("Turret/TargetPositionDeg", 1.0);
            },
            extender,
            intake,
            turret,
            flywheel),
        Commands.parallel(
            Commands.sequence(
                Commands.waitSeconds(0.02),
                Commands.runOnce(() -> SmartDashboard.putNumber("Turret/TargetPositionDeg", 0.0)),
                Commands.waitUntil(() -> turret != null && turret.atTarget()).withTimeout(1.0),
                Commands.runOnce(() -> driverTurretOverride.accept(false))),
            Commands.sequence(
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> extender.setRetractedState(), extender),
                Commands.runOnce(() -> intake.setIdleState(), intake))));
  }
}
