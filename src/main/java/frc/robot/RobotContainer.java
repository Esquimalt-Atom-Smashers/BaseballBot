// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Mass;
import org.dyn4j.geometry.Rectangle;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SafeRetractExtenderCommand;
import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import static frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.extender.*;
import frc.robot.subsystems.agitator.*;
import frc.robot.subsystems.candle.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.transfer.*;
import frc.robot.subsystems.shooter.turret.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.hang.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// Controller
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandXboxController operatorController = new CommandXboxController(1);

	// Competition Toggle
	@AutoLogOutput(key = "CompetitionToggle")
	private boolean isCompetition = true;

	// Subsystems Toggle
	private boolean isDriveEnabled 		= false;
	private boolean isVisionEnabled 	= false;
	private boolean isIntakeEnabled 	= false;
	private boolean isExtenderEnabled = false;
	private boolean isAgitatorEnabled = false;
	private boolean isTransferEnabled = false;
	private boolean isTurretEnabled 	= false;
	private boolean isHoodEnabled 		= false;
	private boolean isFlywheelEnabled = false;
	private boolean isCandleEnabled 	= true;
	private boolean isHangEnabled 		= false;

	// Simulation Toggle
	private boolean halfFuelOnly 			= true;
	private boolean shooterSimEnabled	= true;
	private boolean fuelSimEnabled 		= true;

	// Subsystems
	private final Drive drive;
	@SuppressWarnings("unused")
	private final Vision vision;
	private final Intake intake;
	private final Extender extender;
	private final Agitator agitator;
	private final Transfer transfer;
	private final Turret turret;
	private final Hood hood;
	private final Flywheel flywheel;
	private final CANdle candle;
	private final Hang hang;

	// Drive Commands
	private final TeleopDrive teleopDrive;

	// Robot Auto-Face Hub Target Mode
	@AutoLogOutput(key = "TeleopDrive/IsFacingHub")
	private boolean isFacingHub = false;
	private ProfiledPIDController faceTargetController;

	// Shooter Manager
	private final Shooter shooter;
	private final ShootWhenReadyCommand shootWhenReadyCommand;

	// Safe Extender Retracter
	private final Command safeRetractExtenderCommand;

	// Field view (robot pose)
	private final Field2d field = new Field2d();

	// Robot-centric vs Field-centric drive
	@AutoLogOutput(key = "TeleopDrive/IsRobotCentric")
	private boolean isRobotCentric = false;

	// Manual Override
	@AutoLogOutput(key = "ManualOverride/Driver")
	public static boolean driverManualOverride = false;
	@AutoLogOutput(key = "ManualOverride/Operator")
	public static boolean operatorManualOverride = false;
	@AutoLogOutput(key = "Subsystems/Shooter/Turret/DriverTurretOverride")
	private boolean driverTurretOverride = false;

	// Hang Hold Mode
	@AutoLogOutput(key = "Subsystems/Hang/HangHoldModeEnabled")
	private boolean hangHoldModeEnabled = false;

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	// Drive Simulation
	private SwerveDriveSimulation driveSimulation = null;
	/** Last extender state used for dyn4j bumper footprint (SIM only). */
	private boolean driveSimCollisionExtenderExtended = false;

	// Fuel Simulation (robot-ball collision in SIM)
	private final FuelSim fuelSim = new FuelSim();

	// Shooter Simulation and Visualizer (only non-null in SIM)
	private final ShooterSim shooterSim;
	private final ShooterSimVisualizer shooterSimVisualizer;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
    // Initialize Subsystems based on mode (REAL, SIM, or REPLAY)
		switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
			case REAL:
				if (isDriveEnabled) {
					drive = new Drive(
							new GyroIOPigeon2(),
							new ModuleIOTalonFX(TunerConstants.FrontLeft), new ModuleIOTalonFX(TunerConstants.FrontRight),
							new ModuleIOTalonFX(TunerConstants.BackLeft), new ModuleIOTalonFX(TunerConstants.BackRight),
							(pose) -> {});
				} else {
					drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
				}
				
				// Initialize Vision after Drive (Vision needs Drive reference)
				if (isVisionEnabled) {
					vision = new Vision(drive,
							new VisionIOPhotonVision(camera0Name, robotToCamera0), 
							new VisionIOPhotonVision(camera1Name, robotToCamera1));
				} else {
					vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
				}

				// Subsystems
				intake   = isIntakeEnabled 	 ? new Intake(new IntakeIOSparkMax()) 	  : new Intake(new IntakeIO() {});
				extender = isExtenderEnabled ? new Extender(new ExtenderIOSparkMax()) : new Extender(new ExtenderIO() {});
				agitator = isAgitatorEnabled ? new Agitator(new AgitatorIOSparkMax()) : new Agitator(new AgitatorIO() {});
				transfer = isTransferEnabled ? new Transfer(new TransferIOSparkMax()) : new Transfer(new TransferIO() {});
				turret   = isTurretEnabled 	 ? new Turret(new TurretIOSparkMax()) 	  : new Turret(new TurretIO() {});
				hood     = isHoodEnabled  	 ? new Hood(new HoodIOSparkMax()) 		  	: new Hood(new HoodIO() {});
				flywheel = isFlywheelEnabled ? new Flywheel(new FlywheelIOTalonFX())  : new Flywheel(new FlywheelIO() {});
				candle 	 = isCandleEnabled   ? new CANdle(new CANdleIOLEDs())  			  : new CANdle(new CANdleIO() {});
				hang 	 	 = isHangEnabled		 ? new Hang(new HangIOSparkMax())  				: new Hang(new HangIO() {});
				shooterSim = null;
				shooterSimVisualizer = null;
				break;

			// Sim robot, instantiate physics sim IO implementations
			case SIM:
        // Configure Simulation timing for accurate physics
        // 5 ticks per 20ms period = 4kHz effective control loop rate
        SimulatedArena.overrideSimulationTimings(
            edu.wpi.first.units.Units.Seconds.of(0.02), // 20ms robot period
            5); // 5 Simulation ticks per period

        // Use local arena with to drive through ramps
        SimulatedArena.overrideInstance(new frc.robot.simulation.Arena2026Rebuilt(true));

				driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
				applyRearGapToDriveCollision(driveSimulation, 0.0);
				SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
				drive = new Drive(
						new GyroIOSim(driveSimulation.getGyroSimulation()),
						new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0], 0),
						new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1], 1),
						new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2], 2),
						new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3], 3),
						driveSimulation::setSimulationWorldPose);
				
				// Initialize Vision after Drive (Vision needs Drive reference)
				vision = new Vision(drive,
						new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
						new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

				// Subsystems
				intake 	 = new Intake(new IntakeIOSim());
				extender = new Extender(new ExtenderIOSim());
				agitator = new Agitator(new AgitatorIOSim());
				transfer = new Transfer(new TransferIOSim());
				turret 	 = new Turret(new TurretIOSim());
				hood 		 = new Hood(new HoodIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());
				candle   = new CANdle(new CANdleIOSim());
				hang 		 = new Hang(new HangIOSim());

				// Shooter Sim Visualizer
				if (shooterSimEnabled) {
					shooterSimVisualizer = new ShooterSimVisualizer(() -> {
							Pose2d simPose = driveSimulation.getSimulatedDriveTrainPose();
							return new Pose3d(
									simPose.getX(),
									simPose.getY(),
									0,
									new Rotation3d(0, 0, simPose.getRotation().getRadians()));
						},
						drive::getFieldRelativeChassisSpeeds);
					shooterSim = new ShooterSim(fuelSim, fuelSimEnabled, shooterSimVisualizer);
				} else {
					shooterSim = null;
					shooterSimVisualizer = null;
				}

				// Fuel Sim
				if (fuelSimEnabled) {
					configureFuelSim();
					Runnable intakeFuelCallback = shooterSim != null ? shooterSim::intakeFuel : () -> {};
					configureFuelSimRobot(
							() ->
									extender.getState() == Extender.State.EXTENDED
											&& (shooterSim == null || shooterSim.canIntake()),
							intakeFuelCallback);
				}
				break;

			// Replayed Robot, disable IO implementations
			default:
				drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
				vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

				// Subsystems
				intake 	 = new Intake(new IntakeIO() {});
				extender = new Extender(new ExtenderIO() {});
				agitator = new Agitator(new AgitatorIO() {});
				transfer = new Transfer(new TransferIO() {});
				turret 	 = new Turret(new TurretIO() {});
				hood 		 = new Hood(new HoodIO() {});
				flywheel = new Flywheel(new FlywheelIO() {});
				candle   = new CANdle(new CANdleIO() {});
				hang 		 = new Hang(new HangIO() {});
				shooterSim = null;
				shooterSimVisualizer = null;
				break;
			}

		// Shooter coordinator and shoot-when-ready command
		shooter = new Shooter(drive, agitator, transfer, turret, hood, flywheel, isHoodEnabled);
		shootWhenReadyCommand = new ShootWhenReadyCommand(agitator, transfer, shooter, () -> drive.getPose());

		shooter.setShootCommandScheduledSupplier(shootWhenReadyCommand::isScheduled);
		shooter.setManualOverrideSupplier(() -> operatorManualOverride);

		safeRetractExtenderCommand =
				SafeRetractExtenderCommand.create(
						shootWhenReadyCommand, flywheel, extender, turret, b -> driverTurretOverride = b);

		// Subsystem Manual Override Ignore Limits Supplier
		intake.setIgnoreLimitsSupplier(() 	-> operatorManualOverride);
		extender.setIgnoreLimitsSupplier(() -> operatorManualOverride);
		agitator.setIgnoreLimitsSupplier(() -> operatorManualOverride);
		transfer.setIgnoreLimitsSupplier(() -> operatorManualOverride);
		hood.setIgnoreLimitsSupplier(() 		-> operatorManualOverride);
		flywheel.setIgnoreLimitsSupplier(() -> operatorManualOverride);
		hang.setIgnoreLimitsSupplier(() 		-> operatorManualOverride);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------------- Drive Commands --------------------------------------		
		/// -------------------------------------------------------------------------------------------
		// Initialize face-target PID controller
		// Rotate so Turret pivot aims at Hub, not Robot center. Same PID constants as DriveCommands.
		faceTargetController = new ProfiledPIDController(DriveCommands.getAngleKp(), 0.0, DriveCommands.getAngleKd(),
		new TrapezoidProfile.Constraints(DriveCommands.getAngleMaxVelocity(), DriveCommands.getAngleMaxAcceleration()));
		faceTargetController.enableContinuousInput(-Math.PI, Math.PI);

		teleopDrive = new TeleopDrive(drive, driverController, () -> isRobotCentric, () -> isFacingHub, faceTargetController);
		teleopDrive.setManualOverrideSupplier(() -> driverManualOverride);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------- Shooter Subsystem Commands --------------------------------
		/// -------------------------------------------------------------------------------------------
		// Turret aims at predicted target; velocity feedforward for spin compensation. Only active if not in manualOverride
		turret.setManualOverrideSupplier(() -> operatorManualOverride || driverTurretOverride);
		turret.setDrive(drive);
		turret.setAimAtTargetSupplier(() -> shootWhenReadyCommand.isScheduled());

		candle.setShootWhenReadySupplier(() -> shootWhenReadyCommand.isScheduled());
		candle.setManualOverrideSupplier(() -> RobotContainer.driverManualOverride || RobotContainer.operatorManualOverride);
		candle.setShooter(shooter);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------------ Logger Dashboard -------------------------------------
		/// -------------------------------------------------------------------------------------------
		// Field view: Robot + Turret so you can see Turret direction in sim
		SmartDashboard.putData("Field", field);

		// Register PathPlanner named commands
		registerCommands();

		// Set up Auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Things to set up if not in competition
		if (!isCompetition) {
			// Set up Swerve Calibration Programs
			DriveCommands.swerveCalibration(autoChooser, drive);

			// Record zeroed Robot components (model_0 Turret, model_1 Extender, model_2 extending-storage) – initial only; updated in updateSimulation()
			Logger.recordOutput("ComponentPoses/Zeroed", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
		}

		// Record zeroed Robot components (model_0 Turret, model_1 Extender) – initial only; updated in updateSimulation()
		Logger.recordOutput("ComponentPoses/Final",
				new Pose3d[] {
					new Pose3d(-0.095, -0.17, 0.31, new Rotation3d(0, 0, 0)), // model_0 Turret
					new Pose3d(0.275, 0, 0.195, new Rotation3d(0, 0, 0)),  // model_1 Extender
					new Pose3d(-0.29635, 0.055, 0.215, new Rotation3d(0, 0, 0))   // model_2 Hang
				}
		);
		

		//candle.setLEDColor(new RGBWColor(new Color(0, 0, 255)));
    // Configure button bindings
    configureDriverBindings();
    configureOperatorBindings(true); // False to disable operator controls
  }

	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------------ Controller Input -----------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
  /** Configure Driver controls. */
  private void configureDriverBindings() {
    drive.setDefaultCommand(teleopDrive);

		// Cycle Extender, starts in Retracted, goes to Extended, and then cycles between Partial and Extended. 
		// If in Manual, goes to Extended.
		driverController.leftTrigger().onTrue(Commands.runOnce(() -> {
			switch (extender.getState()) {
				case EXTENDED ->  extender.setPartialState();
				case PARTIAL -> 	extender.setExtendedState();
				case RETRACTED -> extender.setExtendedState();
				case MANUAL -> 		extender.setExtendedState();
				case IDLE ->			extender.setExtendedState();
				default -> throw new IllegalArgumentException("Unexpected value: " + extender.getState());
			}

			// Disable Intake when Extender is Retracted or Partial
			if (extender.getState() == Extender.State.RETRACTED || extender.getState() == Extender.State.PARTIAL) {
				intake.setIdleState();
			}
		}, extender));

		// Set to Retracted, must turn off AutoShoot, and set Turret Target to 0
		driverController.povDown().onTrue(safeRetractExtenderCommand);

		// Intake toggle: right bumper = Intaking ↔ Idle, left bumper = Reversing ↔ Idle
		driverController.leftBumper().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.setIdleState(), intake),
				Commands.runOnce(() -> intake.setReversingState(), intake),
				() -> intake.getState() == Intake.State.REVERSING));
		driverController.rightBumper().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.setIdleState(), intake),
				Commands.runOnce(() -> intake.setIntakingState(), intake),
				() -> intake.getState() == Intake.State.INTAKING));

    // Toggle face-target mode when Y button is pressed
    driverController.y().onTrue(Commands.runOnce(() -> {
      isFacingHub = !isFacingHub;
      if (isFacingHub) {
        // Reset PID controller when enabling face-target mode
        faceTargetController.reset(drive.getRotation().getRadians());
      }
    }, drive));

		// Enable Hang/ Retract mode, stop when released
		if (hang != null) {
			// If hangHoldModeEnabled (set by X in endgame), press → Level 1 and release → Idle; 
			// Else toggle Level 1 / Idle.
			driverController.b().onTrue(
				new ConditionalCommand(
					Commands.runOnce(() -> hang.setLevel1State(), hang),
					new ConditionalCommand(
						Commands.runOnce(() -> hang.setIdleState(), hang),
						Commands.runOnce(() -> hang.setLevel1State(), hang),
						() -> hang.getState() == Hang.State.LEVEL_1),
					() -> hangHoldModeEnabled));
			driverController.b().onFalse(
				new ConditionalCommand(
					Commands.runOnce(() -> hang.setIdleState(), hang),
					new InstantCommand(),
					() -> hangHoldModeEnabled));
			// Before Endgame, X toggles Stored / Idle. 
			// Endgame:        X onTrue sets hangHoldModeEnabled (see B); Hanging state while held, Idle on release
			driverController.x().onTrue(
				new ConditionalCommand(
					Commands.runOnce(() -> hangHoldModeEnabled = true),
					Commands.runOnce(
						() -> {
							if (hang.getState() == Hang.State.STORED) {
								hang.setIdleState();
							} else {
								hang.setStoredState();
							}
						},
						hang),
					() -> isHangDriverEndgamePeriod()));
			driverController.x().whileTrue(
				new ConditionalCommand(
					Commands.startEnd(
						() -> hang.setHangingState(),
						() -> hang.setIdleState(),
						hang),
					new InstantCommand(),
					() -> isHangDriverEndgamePeriod()));
		}

    // Shoot toggle: on = schedule ShootWhenReadyCommand, set Flywheel to Charging if Idle; off = cancel (command end() sets Transfer and Agitator to Idle)
		driverController.a().onTrue(Commands.runOnce(() -> {
			if (shootWhenReadyCommand.isScheduled()) {
				CommandScheduler.getInstance().cancel(shootWhenReadyCommand);
				if (flywheel != null ) flywheel.setState(Flywheel.State.IDLE);
			} else {
				if (flywheel != null && flywheel.getState() == Flywheel.State.IDLE) {
					flywheel.setState(Flywheel.State.CHARGING);
				}
				CommandScheduler.getInstance().schedule(shootWhenReadyCommand);
			}
		}));

		// Reset Gyro / Odometry, or if Manual Override is true, Reset Gyro
		final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
        : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // Zero Gyro
		driverController.start().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> isRobotCentric = !isRobotCentric, drive),
				Commands.runOnce(resetGyro, drive).ignoringDisable(true),
				() -> !driverManualOverride));


    // -------- Auto Pathfind to Target --------
    // Pathfind then follow path to outpost
    driverController.leftStick().whileTrue(DriveCommands.pathfindThenFollowPath(drive, "GoTo-Outpost"));

    driverController.povLeft()
        .whileTrue(Commands.defer(() -> hangAssistAfterPathCommand("Hang-HangingLeft"), Set.of(drive, hang)));
    driverController.povRight()
        .whileTrue(Commands.defer(() -> hangAssistAfterPathCommand("Hang-HangingRight-TeleOp"), Set.of(drive, hang)));

		// ------------------------------------------- Driver Manual Override -------------------------------------------
		// If Manual Override is false, become true. 
		// If true, reset encoder positions and then become false.
		driverController.back().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> driverManualOverride = false),
				Commands.runOnce(() -> driverManualOverride = true),
				() -> driverManualOverride));
  } // End configureDriverBindings

  /** Configure Operator controls. */
  private void configureOperatorBindings(boolean enableOperatorControls) {
    // Operator Controls Enabled
    if (!enableOperatorControls) {
			return;
		}

		// Intake Manual Voltage Control
		// Raise Intake voltage
		operatorController.povLeft().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.stepVoltage(IntakeConstants.kStepVolts), intake),
				new InstantCommand(),
				() -> intake != null
			)
		);
		// Lower Intake voltage
		operatorController.povRight().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.stepVoltage(-IntakeConstants.kStepVolts), intake),
				new InstantCommand(),
				() -> intake != null
			)
		);

		// Extender Manual Position Control
		// Raise Extender Position
		operatorController.leftTrigger().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					extender.stepPositionRad(ExtenderConstants.kStepRad);
				}),
				new InstantCommand(),
				() -> extender != null)
		);
		// Lower Extender Position
		operatorController.rightTrigger().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					extender.stepPositionRad(-ExtenderConstants.kStepRad);
				}),
				new InstantCommand(),
				() -> extender != null)
		);

		// Hang Manual Position Control
		// Raise Hang (Extend)
		operatorController.b().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> hang.stepPositionMeters(HangConstants.kStepMeters), hang),
				new InstantCommand(),
				() -> hang != null
			)
		);
		// Lower Hang (Retract)
		operatorController.x().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> hang.stepPositionMeters(-HangConstants.kStepMeters), hang),
				new InstantCommand(),
				() -> hang != null
			)
		);


		// ------------------------------------------ Operator Manual Override ------------------------------------------
		// If Manual Override is false, become true. 
		// If true, reset encoder positions and then become false.
		operatorController.back().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> operatorManualOverride = false),
				Commands.runOnce(() -> operatorManualOverride = true),
				() -> operatorManualOverride));

		// Agitator Manual Voltage Control
		// Raise Agitator voltage
		operatorController.y().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> agitator.stepVoltage(AgitatorConstants.kStepVolts), agitator),
				new InstantCommand(),
				() -> (operatorManualOverride && agitator != null)
			)
		);
		// Lower Agitator voltage
		operatorController.a().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> agitator.stepVoltage(-AgitatorConstants.kStepVolts), agitator),
				new InstantCommand(),
				() -> (operatorManualOverride && agitator != null)
			)
		);

		// Transfer Manual Voltage Control
		// Raise Transfer voltage
		operatorController.leftBumper().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> transfer.stepVoltage(TransferConstants.kStepVolts), transfer),
				new InstantCommand(),
				() -> (operatorManualOverride && transfer != null)
			)
		);
		// Lower Transfer voltage
		operatorController.rightBumper().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> transfer.stepVoltage(-TransferConstants.kStepVolts), transfer),
				new InstantCommand(),
				() -> (operatorManualOverride && transfer != null)
			)
		);
		
		// Turret Manual Position Control
		// Step Turret position up
		operatorController.leftStick().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> turret.stepPositionRad(TurretConstants.kStepRad), turret),
				new InstantCommand(),
				() -> (operatorManualOverride && turret != null)
			)
		);
		// Step Turret position down
		operatorController.rightStick().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> turret.stepPositionRad(-TurretConstants.kStepRad), turret),
				new InstantCommand(),
				() -> (operatorManualOverride && turret != null)
			)
		);
		// Reset Extender and Turret Encoder, and Idle all subsystems.
		operatorController.start().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					extender.resetEncoders();
					turret.resetMotorEncoder();
					hang.resetEncoders();
					idleAllSubsystems();
				}, turret, extender),
				new InstantCommand(),
				() -> (operatorManualOverride && turret != null)
			)
		);

		// Flywheel Manual Velocity Control
		// Raise Flywheel RPM
		operatorController.povUp().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> flywheel.stepVelocityRadPerSec(FlywheelConstants.kStepRadPerSec), flywheel),
				new InstantCommand(),
				() -> (operatorManualOverride && flywheel != null)
			)
		);
		// Lower Flywheel RPM
		operatorController.povDown().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> flywheel.stepVelocityRadPerSec(-FlywheelConstants.kStepRadPerSec), flywheel),
				new InstantCommand(),
				() -> (operatorManualOverride && flywheel != null)
			)
		);
  } // End configureOperatorBindings


	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------- Autonomous Commands Only --------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
	/** Register the commands here */
	private void registerCommands() {
		// Intake Commands
		NamedCommands.registerCommand("Intake On", Commands.runOnce(() -> intake.setIntakingState(), intake));
		NamedCommands.registerCommand("Intake Off", Commands.runOnce(() -> intake.setIdleState(), intake));
		NamedCommands.registerCommand("Intake Reverse", Commands.runOnce(() -> intake.setReversingState(), intake));

		NamedCommands.registerCommand("Extender Down", Commands.runOnce(() -> extender.setExtendedState(), extender));
		NamedCommands.registerCommand("Extender Partial", Commands.runOnce(() -> extender.setPartialState(), extender));
		NamedCommands.registerCommand("Extender Up", Commands.runOnce(() -> extender.setRetractedState(), extender));
		NamedCommands.registerCommand("Safe Retract Extender", safeRetractExtenderCommand);

		// Flywheel Commands
		NamedCommands.registerCommand("Flywheel On", Commands.runOnce(() -> flywheel.setState(Flywheel.State.CHARGING), flywheel));
		NamedCommands.registerCommand("Flywheel Off", Commands.runOnce(() -> flywheel.setState(Flywheel.State.IDLE), flywheel));

		// Shooter Target Commands
		NamedCommands.registerCommand("Set Shooter Target Hub", Commands.runOnce(ShooterCommands::clearShooterTargetOverride));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Left", Commands.runOnce(ShooterCommands::setPassingSpotLeft));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Center", Commands.runOnce(ShooterCommands::setPassingSpotCenter));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Right", Commands.runOnce(ShooterCommands::setPassingSpotRight));
		// With timeout so the sequential auto can advance to path commands (reference codebases build autos in code with paths only)
		NamedCommands.registerCommand("Shoot When Ready", Commands.runOnce(() -> CommandScheduler.getInstance().schedule(shootWhenReadyCommand)));
		NamedCommands.registerCommand("Cancel Shoot When Ready", Commands.runOnce(() -> CommandScheduler.getInstance().cancel(shootWhenReadyCommand)));

		// Hang Commands
		NamedCommands.registerCommand("Hang Level 1", Commands.runOnce(() -> hang.setLevel1State(), hang));
		NamedCommands.registerCommand("Hang Hanging", Commands.runOnce(() -> hang.setHangingState(), hang));
		NamedCommands.registerCommand("Hang Stored", Commands.runOnce(() -> hang.setStoredState(), hang));

		// Drive Commands (auto)
		NamedCommands.registerCommand("Drive Back Hang Align", DriveCommands.timedDriveBackRobotCentric(drive));
	} // End registerCommands

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	} // End getAutonomousCommand

	/// -----------------------------------------------------------------------------------------------------------------
	/// --------------------------------------------- Other Useful Methods ----------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
	/** Idle all Subsystems.  */
	public void idleAllSubsystems() {
		CommandScheduler.getInstance().cancel(shootWhenReadyCommand);
		if (intake != null)		intake.setIdleState();
		if (extender != null) extender.setIdleState();
		if (agitator != null) agitator.setIdleState();
		if (transfer != null) transfer.setIdleState();
		if (flywheel != null) flywheel.setState(Flywheel.State.IDLE);
		if (hang != null) 		hang.setIdleState();
	} // End idleBallHandling

	/**
	 * Check if we are in the last 30 seconds (Endgame).
	 * Requires FMS so practice mode (match time often 0) does not always select endgame.
	 */
	private boolean isHangDriverEndgamePeriod() {
		return DriverStation.isTeleop()
				&& DriverStation.isFMSAttached()
				&& DriverStation.getMatchTime() <= Constants.MatchTiming.HANG_DRIVER_ENDGAME_SECONDS;
	}
	
  /**
   * Hang assist: set hang to {@link Hang.State#LEVEL_1}, pathfind and follow the named PathPlanner
   * path, run the same short robot-centric backup used for hang alignment in autos, then set hang to
   * {@link Hang.State#HANGING}.
   */
  private Command hangAssistAfterPathCommand(String pathName) {
    return Commands.sequence(
        Commands.runOnce(() -> hang.setLevel1State(), hang),
        DriveCommands.pathfindThenFollowPath(drive, pathName),
        DriveCommands.timedDriveBackRobotCentric(drive));
  }

  /**
   * Update Field2d with the current robot pose. Call from Robot.robotPeriodic().
   * In SIM mode the pose is set in updateSimulation(); on real robot we use drive odometry.
   */
	public void updateFieldPose() {
		if (Constants.currentMode == Constants.Mode.SIM) return;
		field.setRobotPose(drive.getPose());
	} // End updateFieldPose


	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------------ Simulation Only ------------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
  /** Reset the simulation field for autonomous mode. Only works in SIM mode. */
	public void resetSimulationField() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
		SimulatedArena.getInstance().resetFieldForAuto();
	} // End resetSimulationField

	/**
	 * Updates the dyn4j chassis collision footprint for the MapleSim drive sim so the rear bumper
	 * has a centered opening. Optional {@code frontExtensionBeyondBumpersM} lengthens the hull in
	 * +X when the extender is extended ({@link Constants.Dimensions#kExtensionPastBumpersMeters}).
	 *
	 * This only affects the drive simulation collision detection (not FuelSim).
	 */
	private static void applyRearGapToDriveCollision(
			SwerveDriveSimulation driveSimulation, double frontExtensionBeyondBumpersM) {
		// MapleSim / dyn4j dimensions are meters.
		double bumperLengthX = Constants.Dimensions.FULL_LENGTH.in(Meters);
		double bumperWidthY = Constants.Dimensions.FULL_WIDTH.in(Meters);

		// Rear gap spec: width = 17" (centered on robot Y=0) and depth = 3" (into the robot in -X).
		double gapHalfWidthY = 8.5 * 0.0254; // 8.5 in half-width
		double gapDepthX = 3.0 * 0.0254; // 3 in depth

		double halfLenX = bumperLengthX / 2.0;
		double halfWidthY = bumperWidthY / 2.0;

		double sideHeightY = halfWidthY - gapHalfWidthY;
		if (sideHeightY <= 0) {
			// Avoid generating invalid fixtures if constants are inconsistent.
			return;
		}

		// Preserve dyn4j mass/inertia (MapleSim applies forces using its original COM assumptions).
		// If we recompute mass from the new U-shape fixtures, the COM shifts forward and the
		// physics can become unstable (robot "haywires" on joystick input).
		Mass oldMass = driveSimulation.getMass().copy();

		// Remove the default bumper rectangle fixtures and replace with 3 rectangles:
		// - a front rectangle (full width) stopping at the rear gap start
		// - a left rear strip
		// - a right rear strip
		driveSimulation.removeAllFixtures();

		// Front rectangle: x in [-halfLenX + gapDepthX, +halfLenX + frontExtensionBeyondBumpersM]
		double frontLeftX = -halfLenX + gapDepthX;
		double frontRightX = halfLenX + frontExtensionBeyondBumpersM;
		double frontWidthX = frontRightX - frontLeftX;
		double frontCenterX = (frontLeftX + frontRightX) * 0.5;
		Rectangle front = new Rectangle(frontWidthX, bumperWidthY);
		front.translate(frontCenterX, 0);
		BodyFixture frontFixture = new BodyFixture(front);
		frontFixture.setFriction(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_FRICTION);
		frontFixture.setRestitution(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_RESTITUTION);
		frontFixture.setDensity(0.0); // collision-only; keep oldMass for dynamics stability
		driveSimulation.addFixture(frontFixture);

		// Left & right strips: x in [-halfLenX, -halfLenX + gapDepthX], y outside the gap
		double stripWidthX = gapDepthX;
		double stripCenterX = -halfLenX + gapDepthX / 2.0;
		double sideCenterY = (gapHalfWidthY + halfWidthY) / 2.0;

		Rectangle left = new Rectangle(stripWidthX, sideHeightY);
		left.translate(stripCenterX, -sideCenterY);
		BodyFixture leftFixture = new BodyFixture(left);
		leftFixture.setFriction(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_FRICTION);
		leftFixture.setRestitution(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_RESTITUTION);
		leftFixture.setDensity(0.0); // collision-only; keep oldMass for dynamics stability
		driveSimulation.addFixture(leftFixture);

		Rectangle right = new Rectangle(stripWidthX, sideHeightY);
		right.translate(stripCenterX, sideCenterY);
		BodyFixture rightFixture = new BodyFixture(right);
		rightFixture.setFriction(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_FRICTION);
		rightFixture.setRestitution(AbstractDriveTrainSimulation.BUMPER_COEFFICIENT_OF_RESTITUTION);
		rightFixture.setDensity(0.0); // collision-only; keep oldMass for dynamics stability
		driveSimulation.addFixture(rightFixture);

		// Restore old mass/inertia so COM stays where MapleSim expects.
		driveSimulation.setMass(oldMass);
	} // End applyRearGapToDriveCollision

  /** Configures FuelSim for robot-ball collision in simulation. */
  private void configureFuelSim() {
    fuelSim.setShowHalfFuel(halfFuelOnly);
		fuelSim.enableAirResistance();
    fuelSim.spawnStartingFuel();

    fuelSim.start();
		SmartDashboard.putData(Commands.runOnce(() -> {
						fuelSim.clearFuel();
						fuelSim.spawnStartingFuel();
				})
				.withName("Reset Fuel")
				.ignoringDisable(true));
  } // End configureFuelSim
	
	/** Configures the robot for fuel simulation. */
	private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
		double robotWidthMeters = Constants.Dimensions.FULL_WIDTH.in(Meters);
		double robotLengthMeters = Constants.Dimensions.FULL_LENGTH.in(Meters);
		double bumperHeightMeters = Constants.Dimensions.BUMPER_HEIGHT.in(Meters);

		// Register a robot for collision with fuel
		fuelSim.registerRobot(
				robotWidthMeters,
				robotLengthMeters,
				bumperHeightMeters,
				drive::getPose,
				drive::getFieldRelativeChassisSpeeds);

		// Register intakes for the robot
		// Intake: 10.5" beyond front of frame (+X), full width minus 2" on each side
		double intakeExtendMeters = 10.5 * 0.0254;
		double intakeInsetMeters = 2.0 * 0.0254;
		fuelSim.registerIntake(
				robotLengthMeters / 2,
				robotLengthMeters / 2 + intakeExtendMeters,
				-robotWidthMeters / 2 + intakeInsetMeters,
				robotWidthMeters / 2 - intakeInsetMeters,
				() -> ableToIntake.getAsBoolean() && intake.getState() == Intake.State.INTAKING,
				intakeCallback);
	} // End configureFuelSimRobot

  /** Update the Simulation world. Should be called from Robot.simulationPeriodic(). Only works in SIM mode. */
	public void updateSimulation() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		boolean extenderExtendedForCollision = extender.getState() == Extender.State.EXTENDED;
		if (extenderExtendedForCollision != driveSimCollisionExtenderExtended) {
			driveSimCollisionExtenderExtended = extenderExtendedForCollision;
			applyRearGapToDriveCollision(
					driveSimulation,
					extenderExtendedForCollision ? Constants.Dimensions.kExtensionPastBumpersMeters : 0.0);
		}

		SimulatedArena.getInstance().simulationPeriodic();

		// Robot pose for visualization
		Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
		Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);

		// Robot-relative component poses for visualization
		Pose3d turretComponentPose 	 = new Pose3d(-0.095, -0.17, 0.31, new Rotation3d(0, 0, turret.getRobotFramePosition().getRadians() - Math.toRadians(90)));
		Pose3d extenderComponentPose = new Pose3d(0.275, 0, 0.195, new Rotation3d(0, extender.getPositionRad() - Math.toRadians(90), 0));
		Pose3d hangComponentPose 		 = new Pose3d(-0.29635, 0.055, 0.215 + hang.getPositionMeters(), new Rotation3d(0, 0, 0)); // Placeholder pose for Hang; update when Hang sim is implemented
		Logger.recordOutput("ComponentPoses/Final", new Pose3d[] {turretComponentPose, extenderComponentPose, hangComponentPose});

		// Update field view
		field.setRobotPose(robotPose);

		// Shooter sim: launch fuel only when Shooter is ready and shoot command is active
		if (shooterSim != null) {
			shooterSim.update(shooter, shooter::isShootCommandActive, turret, hood, flywheel);
		}
		if (shooterSimVisualizer != null) {
			double hoodAngleRad = isHoodEnabled ? hood.getAngleRad() : HoodConstants.kDisabledAngleRad;
			double flywheelSurfaceMps = flywheel.getTargetVelocityRadPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
			double ballExitVelMps = flywheelSurfaceMps * ShooterConstants.kFlywheelSurfaceDivider;
			shooterSimVisualizer.updateFuel(
					edu.wpi.first.units.Units.MetersPerSecond.of(ballExitVelMps),
					edu.wpi.first.units.Units.Radians.of(hoodAngleRad),
					edu.wpi.first.units.Units.Radians.of(turret.getRobotFramePosition().getRadians()));
			shooterSimVisualizer.update3dPose(
					edu.wpi.first.units.Units.Radians.of(turret.getRobotFramePosition().getRadians()),
					edu.wpi.first.units.Units.Radians.of(hoodAngleRad));
		}

		// Fuel sim (robot-ball collision)
		if (fuelSimEnabled) {
			fuelSim.updateSim();
		}

		// Log balls in robot and hub scores (FuelSim)
		if (shooterSim != null) {
			Logger.recordOutput("FuelSim/BallsInRobot", shooterSim.getFuelStored());
		}
		if (fuelSimEnabled) {
			Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
			Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
		}
	} // End updateSimulation
}
