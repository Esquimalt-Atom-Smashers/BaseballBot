package frc.robot.simulation;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.shooter.ShooterSimVisualizer;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.extender.Extender;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.transfer.Transfer;

/** Maple sim + subsystems for the optional second simulated robot (driver OI). */
public final class SecondSimRobotBundle {

	public SwerveDriveSimulation driveSimulation;
	public Drive drive;
	public Intake intake;
	public Extender extender;
	public Agitator agitator;
	public Transfer transfer;
	public Turret turret;
	public Hood hood;
	public Flywheel flywheel;
	public Hang hang;
	public ShooterSim shooterSim;
	public ShooterSimVisualizer shooterSimVisualizer;
	public ProfiledPIDController faceTargetController;
	public CommandXboxController driverController;
	public TeleopDrive teleopDrive;
	public Shooter shooter;
	public ShootWhenReadyCommand shootWhenReady;
	public Command safeRetractExtenderCommand;
	public boolean driveSimCollisionExtenderExtended;
	public boolean isFacingHub;
	public boolean isRobotCentric;
	public boolean driverTurretOverride;
} // End SecondSimRobotBundle
