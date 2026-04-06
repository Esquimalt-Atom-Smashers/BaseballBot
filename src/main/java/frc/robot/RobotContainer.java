// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

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
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import static frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.extender.*;
import frc.robot.subsystems.agitator.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.transfer.*;
import frc.robot.subsystems.shooter.turret.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.hang.*;

import frc.robot.simulation.SecondSimRobotOutputs;


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
	private boolean isDriveEnabled 		= true;
	private boolean isVisionEnabled 	= true;
	private boolean isIntakeEnabled 	= true;
	private boolean isExtenderEnabled = true;
	private boolean isAgitatorEnabled = true;
	private boolean isTransferEnabled = true;
	private boolean isTurretEnabled 	= true;
	private boolean isHoodEnabled 		= false;
	private boolean isFlywheelEnabled = true;
	private boolean isHangEnabled 		= true;

	// Simulation Toggle
	private boolean halfFuelOnly 			= true;
	private boolean shooterSimEnabled	= true;
	private boolean fuelSimEnabled 		= true;

	// Second Sim Robot Toggle
	private static final boolean kSecondSimRobotEnabled 		= true;
	private static final boolean kSecondSimRobotRedAlliance = true;

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

	// Second Sim Robot
	/** USB port for second sim driver gamepad (mirrors port 0 bindings only). */
	private static final int kSecondSimDriverControllerPort = 3;

	/** Full duplicate sim robot (driver-only OI); null unless SIM and {@link #kSecondSimRobotEnabled}. */
	private SecondSimRobot secondSimRobot = null;


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
				createRobotShape(driveSimulation, 0.0);
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
				hang 		 = new Hang(new HangIOSim());

				int fuelRobotIndex = 0;
				if (fuelSimEnabled) {
					fuelRobotIndex = registerFuelSimRobotBody(drive, false, driveSimulation);
				}

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
					shooterSim = new ShooterSim(fuelSim, fuelSimEnabled ? fuelRobotIndex : 0,
							fuelSimEnabled && shooterSimEnabled, shooterSimVisualizer);
				} else {
					shooterSim = null;
					shooterSimVisualizer = null;
				}

				// Fuel Sim
				if (fuelSimEnabled) {
					configureFuelSim();
					Runnable intakeFuelCallback = shooterSim != null ? shooterSim::intakeFuel : () -> {};
					registerFuelSimIntake(fuelRobotIndex, intake, () ->
							extender.getState() == Extender.State.EXTENDED && (shooterSim == null || shooterSim.canIntake()),
							intakeFuelCallback);
				}
				
				// Second Sim Robot
				if (kSecondSimRobotEnabled) {
					secondSimRobot = buildSecondSimRobot();
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
				});
			

    // Configure button bindings
    configureDriverBindings();
    configureOperatorBindings(true); // False to disable operator controls


		// Second Sim Robot Setup
		if (secondSimRobot != null) {
			secondSimRobot.shooter = new Shooter(secondSimRobot.drive, secondSimRobot.agitator, secondSimRobot.transfer,
					secondSimRobot.turret, secondSimRobot.hood, secondSimRobot.flywheel, isHoodEnabled, SecondSimRobotOutputs.LOG_ROOT_PREFIX);
			secondSimRobot.shootWhenReady = new ShootWhenReadyCommand(secondSimRobot.agitator, secondSimRobot.transfer, secondSimRobot.shooter, secondSimRobot.drive::getPose);
			
			secondSimRobot.shooter.setShootCommandScheduledSupplier(secondSimRobot.shootWhenReady::isScheduled);
			secondSimRobot.shooter.setManualOverrideSupplier(() -> operatorManualOverride);
			secondSimRobot.shooter.setIsRedAllianceSupplier(() -> kSecondSimRobotRedAlliance);
			ShooterCommands.registerTargetAllianceSupplier(secondSimRobot.drive, () -> kSecondSimRobotRedAlliance);
			
			secondSimRobot.safeRetractExtenderCommand =
					SafeRetractExtenderCommand.create(
							secondSimRobot.shootWhenReady, secondSimRobot.flywheel, secondSimRobot.extender, secondSimRobot.turret, b -> secondSimRobot.driverTurretOverride = b);
			
			secondSimRobot.turret.setManualOverrideSupplier(() -> operatorManualOverride || secondSimRobot.driverTurretOverride);
			secondSimRobot.turret.setDrive(secondSimRobot.drive);
			secondSimRobot.turret.setAimAtTargetSupplier(() -> secondSimRobot.shootWhenReady.isScheduled());

			configureDriverBindings(createSecondSimDriverBindParams());
		}
  } // End RobotContainer Constructor

	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------------ Controller Input -----------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
  /** Configure Driver controls. */
	private void configureDriverBindings(DriverBindParams bind) {
		CommandXboxController driverController = bind.driverController();
		Drive drive = bind.drive(); TeleopDrive teleopDrive = bind.teleopDrive();
		Intake intake = bind.intake(); Extender extender = bind.extender(); Flywheel flywheel = bind.flywheel(); Hang hang = bind.hang();
		ShootWhenReadyCommand shootWhenReadyCommand = bind.shootWhenReadyCommand(); ProfiledPIDController faceTargetController = bind.faceTargetController();
		Command safeRetractExtenderCommand = bind.safeRetractExtenderCommand();
		Runnable resetGyro = bind.resetGyro();
		BooleanSupplier facingHubGetter = bind.facingHubGetter(); Consumer<Boolean> facingHubSetter = bind.facingHubSetter();
		BooleanSupplier robotCentricGetter = bind.robotCentricGetter(); Consumer<Boolean> robotCentricSetter = bind.robotCentricSetter();
		BooleanSupplier notDriverManualOverride = bind.notDriverManualOverride();
		boolean bindPrimaryOnlyExtras = bind.bindPrimaryOnlyExtras();

		drive.setDefaultCommand(teleopDrive);

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

		// Toggle face-target mode when Y button is pressed
		driverController.y().onTrue(Commands.runOnce(() -> {
			boolean nextFacingHub = !facingHubGetter.getAsBoolean();
			facingHubSetter.accept(nextFacingHub);
			// Reset PID controller when enabling face-target mode
			if (nextFacingHub) {
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
					Commands.runOnce(() -> {
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
		driverController.start().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> robotCentricSetter.accept(!robotCentricGetter.getAsBoolean()), drive),
				Commands.runOnce(resetGyro, drive).ignoringDisable(true),
				notDriverManualOverride));

		// -------- Auto Pathfind to Target --------
		// Pathfind then follow path to outpost (Only for primary robot)
		if (bindPrimaryOnlyExtras) {
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
		}
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

	/** Inputs for {@link #configureDriverBindings(DriverBindParams)}. */
	private record DriverBindParams(
			CommandXboxController driverController,
			Drive drive, TeleopDrive teleopDrive,
			Intake intake, Extender extender, Flywheel flywheel, Hang hang,
			ShootWhenReadyCommand shootWhenReadyCommand, ProfiledPIDController faceTargetController,
			Command safeRetractExtenderCommand,
			Runnable resetGyro,
			BooleanSupplier facingHubGetter, Consumer<Boolean> facingHubSetter,
			BooleanSupplier robotCentricGetter, Consumer<Boolean> robotCentricSetter,
			BooleanSupplier notDriverManualOverride,
			boolean bindPrimaryOnlyExtras) {
	} // End DriverBindParams

  /** Configure Driver bindings for primary robot. */
	private void configureDriverBindings() {
		configureDriverBindings(createPrimaryDriverBindParams());
	} // End configureDriverBindings

	/** Create Driver bindings for primary robot. */
	private DriverBindParams createPrimaryDriverBindParams() {
		final Runnable resetGyro =
				Constants.currentMode == Constants.Mode.SIM
						? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
						: () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
		return new DriverBindParams(
				driverController, 
				drive, teleopDrive,
				intake, extender, flywheel, hang,
				shootWhenReadyCommand, faceTargetController,
				safeRetractExtenderCommand,
				resetGyro,
				() -> isFacingHub, enabled -> isFacingHub = enabled,
				() -> isRobotCentric, enabled -> isRobotCentric = enabled,
				() -> !driverManualOverride,
				true);
	} // End createPrimaryDriverBindParams

	/** Create Driver bindings for second sim robot. */
	private DriverBindParams createSecondSimDriverBindParams() {
		SecondSimRobot sim2 = secondSimRobot;
		Runnable resetGyro = () -> sim2.drive.setPose(sim2.driveSimulation.getSimulatedDriveTrainPose());
		return new DriverBindParams(
				sim2.driverController, 
				sim2.drive, sim2.teleopDrive,
				sim2.intake, sim2.extender, sim2.flywheel, sim2.hang,
				sim2.shootWhenReady, sim2.faceTargetController,
				sim2.safeRetractExtenderCommand,
				resetGyro,
				() -> sim2.isFacingHub, enabled -> sim2.isFacingHub = enabled,
				() -> sim2.isRobotCentric, enabled -> sim2.isRobotCentric = enabled,
				() -> true,
				false);
	} // End createSecondSimDriverBindParams

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

		// Second Sim Robot
		if (secondSimRobot != null) {
			CommandScheduler.getInstance().cancel(secondSimRobot.shootWhenReady);
			if (secondSimRobot.intake != null)	 secondSimRobot.intake.setIdleState();
			if (secondSimRobot.extender != null) secondSimRobot.extender.setIdleState();
			if (secondSimRobot.agitator != null) secondSimRobot.agitator.setIdleState();
			if (secondSimRobot.transfer != null) secondSimRobot.transfer.setIdleState();
			if (secondSimRobot.flywheel != null) secondSimRobot.flywheel.setState(Flywheel.State.IDLE);
			if (secondSimRobot.hang != null) 		 secondSimRobot.hang.setIdleState();
		}
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
		
		if (secondSimRobot != null) {
			secondSimRobot.driveSimulation.setSimulationWorldPose(new Pose2d(13.5, 5.0, new Rotation2d(Math.PI)));
		}
		SimulatedArena.getInstance().resetFieldForAuto();
	} // End resetSimulationField

	/**
	 * Updates the dyn4j chassis collision footprint for the MapleSim drive sim so the rear bumper
	 * has a centered opening. Optional {@code frontExtensionBeyondBumpersM} lengthens the hull in
	 * +X when the extender is extended ({@link Constants.Dimensions#kExtensionPastBumpersMeters}).
	 *
	 * This only affects the drive simulation collision detection (not FuelSim).
	 */
	private static void createRobotShape(
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
    fuelSim.spawnStartingFuel();

    fuelSim.start();
		SmartDashboard.putData(Commands.runOnce(() -> {
						fuelSim.clearFuel();
						fuelSim.spawnStartingFuel();
				})
				.withName("Reset Fuel")
				.ignoringDisable(true));
  } // End configureFuelSim

	/**
	 * Register a robot for collision with fuel.
	 *
	 * @param mapleDriveSimulation Maple dyn4j chassis for that robot; FuelSim applies opposing impulses when fuel is
	 *     depenetrated (must be the same instance used for pose/speed suppliers’ sim)
	 */
	private int registerFuelSimRobotBody(Drive driveForFuel, boolean additionalRobotSlot, SwerveDriveSimulation mapleDriveSimulation) {
		double robotWidthMeters = Constants.Dimensions.FULL_WIDTH.in(Meters);
		double robotLengthMeters = Constants.Dimensions.FULL_LENGTH.in(Meters);
		double bumperHeightMeters = Constants.Dimensions.BUMPER_HEIGHT.in(Meters);

		// Register a robot for collision with fuel
		if (additionalRobotSlot) {
			return fuelSim.addRegisteredRobot(
					robotWidthMeters,
					robotLengthMeters,
					bumperHeightMeters,
					driveForFuel::getPose,
					driveForFuel::getFieldRelativeChassisSpeeds, mapleDriveSimulation);
		}
		fuelSim.registerRobot(
				robotWidthMeters,
				robotLengthMeters,
				bumperHeightMeters,
				driveForFuel::getPose,
				driveForFuel::getFieldRelativeChassisSpeeds, mapleDriveSimulation);
		return 0;
	} // End registerFuelSimRobotBody

	/** Register an intake for the fuel robot. */
	private void registerFuelSimIntake(
			int fuelRobotIndex, Intake intakeForFuel, BooleanSupplier ableToIntake, Runnable intakeCallback) {
		double robotWidthMeters = Constants.Dimensions.FULL_WIDTH.in(Meters);
		double robotLengthMeters = Constants.Dimensions.FULL_LENGTH.in(Meters);

		// Register intakes for the robot
		// Intake: 10.5" beyond front of frame (+X), full width minus 2" on each side
		double intakeExtendMeters = 10.5 * 0.0254;
		double intakeInsetMeters = 2.0 * 0.0254;
		fuelSim.registerIntake(
				fuelRobotIndex,
				robotLengthMeters / 2,
				robotLengthMeters / 2 + intakeExtendMeters,
				-robotWidthMeters / 2 + intakeInsetMeters,
				robotWidthMeters / 2 - intakeInsetMeters,
				() -> ableToIntake.getAsBoolean() && intakeForFuel.getState() == Intake.State.INTAKING,
				intakeCallback);
	} // End registerFuelSimIntake

	/** Robot-relative component poses for AdvantageScope Simulation. */
	private static Pose3d[] buildComponentPoses(Turret turret, Extender extender, Hang hang) {
		return new Pose3d[] {
			new Pose3d(-0.095, -0.17, 0.31, new Rotation3d(0, 0, turret.getRobotFramePosition().getRadians() - Math.toRadians(90))),
			new Pose3d(0.275, 0, 0.195, new Rotation3d(0, extender.getPositionRad() - Math.toRadians(90), 0)),
			new Pose3d(-0.29635, 0.055, 0.215 + hang.getPositionMeters(), new Rotation3d(0, 0, 0)), // Placeholder pose for Hang; update when Hang sim is implemented
		};
	} // End buildComponentPoses

  /** Update the Simulation world. Should be called from Robot.simulationPeriodic(). Only works in SIM mode. */
	public void updateSimulation() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		boolean extenderExtendedForCollision = extender.getState() == Extender.State.EXTENDED;
		if (extenderExtendedForCollision != driveSimCollisionExtenderExtended) {
			driveSimCollisionExtenderExtended = extenderExtendedForCollision;
			createRobotShape(
					driveSimulation,
					extenderExtendedForCollision ? Constants.Dimensions.kExtensionPastBumpersMeters : 0.0);
		}

		if (secondSimRobot != null) {
			boolean extenderExtendedForCollisionSim2 = secondSimRobot.extender.getState() == Extender.State.EXTENDED;
			if (extenderExtendedForCollisionSim2 != secondSimRobot.driveSimCollisionExtenderExtended) {
				secondSimRobot.driveSimCollisionExtenderExtended = extenderExtendedForCollisionSim2;
				createRobotShape(
						secondSimRobot.driveSimulation,
						extenderExtendedForCollisionSim2 ? Constants.Dimensions.kExtensionPastBumpersMeters : 0.0);
			}
		}

		SimulatedArena.getInstance().simulationPeriodic();

		// Robot pose for visualization
		Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
		Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);
		
		if (secondSimRobot != null) {
			Pose2d robotPoseSim2 = secondSimRobot.driveSimulation.getSimulatedDriveTrainPose();
			Logger.recordOutput(SecondSimRobotOutputs.LOG_ROOT_PREFIX + "FieldSimulation/RobotPosition", robotPoseSim2);
			Logger.recordOutput(SecondSimRobotOutputs.LOG_ROOT_PREFIX + "TeleopDrive/IsFacingHub", secondSimRobot.isFacingHub);
			Logger.recordOutput(SecondSimRobotOutputs.LOG_ROOT_PREFIX + "TeleopDrive/IsRobotCentric", secondSimRobot.isRobotCentric);
			field.getObject("robotPoseSim2").setPose(robotPoseSim2);
		}

		Logger.recordOutput("ComponentPoses/Final", buildComponentPoses(turret, extender, hang));
		if (secondSimRobot != null) {
			Logger.recordOutput(
					SecondSimRobotOutputs.LOG_ROOT_PREFIX + "ComponentPoses/Final",
					buildComponentPoses(secondSimRobot.turret, secondSimRobot.extender, secondSimRobot.hang));
		}

		// Update field view
		field.setRobotPose(robotPose);

		// Shooter sim: launch fuel only when Shooter is ready and shoot command is active
		if (shooterSim != null) {
			shooterSim.update(shooter, shooter::isShootCommandActive, turret, hood, flywheel);
		}
		if (secondSimRobot != null && secondSimRobot.shooterSim != null) {
			secondSimRobot.shooterSim.update(secondSimRobot.shooter, secondSimRobot.shooter::isShootCommandActive, secondSimRobot.turret, secondSimRobot.hood, secondSimRobot.flywheel);
		}
		if (shooterSimVisualizer != null) {
			double hoodAngleRad = isHoodEnabled ? hood.getAngleRad() : HoodConstants.kDisabledAngleRad;
			double flywheelSurfaceMps = flywheel.getTargetVelocityRadPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
			double ballExitVelMps = flywheelSurfaceMps
					* ShooterConstants.kFlywheelSurfaceDivider
					* ShooterConstants.kSimFlywheelToFuelExitVelocityEfficiency;
			shooterSimVisualizer.updateFuel(
					edu.wpi.first.units.Units.MetersPerSecond.of(ballExitVelMps),
					edu.wpi.first.units.Units.Radians.of(hoodAngleRad),
					edu.wpi.first.units.Units.Radians.of(turret.getRobotFramePosition().getRadians()));
			shooterSimVisualizer.update3dPose(
					edu.wpi.first.units.Units.Radians.of(turret.getRobotFramePosition().getRadians()),
					edu.wpi.first.units.Units.Radians.of(hoodAngleRad));
		}
		if (secondSimRobot != null && secondSimRobot.shooterSimVisualizer != null) {
			double hoodAngleRadSim2 = isHoodEnabled ? secondSimRobot.hood.getAngleRad() : HoodConstants.kDisabledAngleRad;
			double flywheelSurfaceMpsSim2 = secondSimRobot.flywheel.getTargetVelocityRadPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
			double ballExitVelMpsSim2 = flywheelSurfaceMpsSim2
					* ShooterConstants.kFlywheelSurfaceDivider
					* ShooterConstants.kSimFlywheelToFuelExitVelocityEfficiency;
			secondSimRobot.shooterSimVisualizer.updateFuel(
					edu.wpi.first.units.Units.MetersPerSecond.of(ballExitVelMpsSim2),
					edu.wpi.first.units.Units.Radians.of(hoodAngleRadSim2),
					edu.wpi.first.units.Units.Radians.of(secondSimRobot.turret.getRobotFramePosition().getRadians()));
			secondSimRobot.shooterSimVisualizer.update3dPose(
					edu.wpi.first.units.Units.Radians.of(secondSimRobot.turret.getRobotFramePosition().getRadians()),
					edu.wpi.first.units.Units.Radians.of(hoodAngleRadSim2));
		}

		// Fuel sim (robot-ball collision)
		if (fuelSimEnabled) {
			fuelSim.updateSim();
		}

		// Log balls in robot and hub scores (FuelSim)
		if (shooterSim != null) {
			Logger.recordOutput("FuelSim/BallsInRobot", shooterSim.getFuelStored());
		}
		if (secondSimRobot != null && secondSimRobot.shooterSim != null) {
			Logger.recordOutput(SecondSimRobotOutputs.LOG_ROOT_PREFIX + "FuelSim/BallsInRobot", secondSimRobot.shooterSim.getFuelStored());
		}
		if (fuelSimEnabled) {
			Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
			Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
		}
	} // End updateSimulation

	/// ------------------------------------------------ Second Sim Robot -----------------------------------------------
	/** Optional second Maple sim + FuelSim robot; built only in SIM when {@link #kSecondSimRobotEnabled}. */
	private static final class SecondSimRobot {
		SwerveDriveSimulation driveSimulation; Drive drive;
		Intake intake; Extender extender; Agitator agitator; Transfer transfer;
		Turret turret; Hood hood; Flywheel flywheel; Hang hang;
		ShooterSim shooterSim; ShooterSimVisualizer shooterSimVisualizer;
		ProfiledPIDController faceTargetController; CommandXboxController driverController;
		TeleopDrive teleopDrive;
		Shooter shooter; ShootWhenReadyCommand shootWhenReady;
		Command safeRetractExtenderCommand; boolean driveSimCollisionExtenderExtended;
		boolean isFacingHub; boolean isRobotCentric;
		boolean driverTurretOverride;
	} // End SecondSimRobot

	private SecondSimRobot buildSecondSimRobot() {
		SecondSimRobot secondSimRobot = new SecondSimRobot();
		secondSimRobot.driverController = new CommandXboxController(kSecondSimDriverControllerPort);

		secondSimRobot.driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(13.5, 5.0, new Rotation2d(Math.PI)));
		createRobotShape(secondSimRobot.driveSimulation, 0.0);
		SimulatedArena.getInstance().addDriveTrainSimulation(secondSimRobot.driveSimulation);
		secondSimRobot.drive = new Drive(
				new GyroIOSim(secondSimRobot.driveSimulation.getGyroSimulation()),
				new ModuleIOSimMapleDirect(TunerConstants.FrontLeft, secondSimRobot.driveSimulation.getModules()[0], 0),
				new ModuleIOSimMapleDirect(TunerConstants.FrontRight, secondSimRobot.driveSimulation.getModules()[1], 1),
				new ModuleIOSimMapleDirect(TunerConstants.BackLeft, secondSimRobot.driveSimulation.getModules()[2], 2),
				new ModuleIOSimMapleDirect(TunerConstants.BackRight, secondSimRobot.driveSimulation.getModules()[3], 3),
				secondSimRobot.driveSimulation::setSimulationWorldPose,
				false,
				Drive.Telemetry.forLogRoot(SecondSimRobotOutputs.LOG_ROOT_PREFIX),
				secondSimRobot.driveSimulation::getSimulatedDriveTrainPose);

		// Second sim robot has no Vision.

		// Subsystems
		secondSimRobot.intake 	= new Intake(new IntakeIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobot.extender = new Extender(new ExtenderIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobot.agitator = new Agitator(new AgitatorIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobot.transfer = new Transfer(new TransferIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobot.turret 	= new Turret(new TurretIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobot.hood 		= new Hood(new HoodIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobot.flywheel = new Flywheel(new FlywheelIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX, SecondSimRobotOutputs.smartDashboardPrefix("Flywheel"));
		secondSimRobot.hang 		= new Hang(new HangIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);

		int fuelRobotIndex = 0;
		if (fuelSimEnabled) {
			fuelRobotIndex = registerFuelSimRobotBody(secondSimRobot.drive, true, secondSimRobot.driveSimulation);
		}

		// Shooter Sim Visualizer
		if (shooterSimEnabled) {
			secondSimRobot.shooterSimVisualizer = new ShooterSimVisualizer(() -> {
					Pose2d simPoseSim2 = secondSimRobot.driveSimulation.getSimulatedDriveTrainPose();
					return new Pose3d(
							simPoseSim2.getX(),
							simPoseSim2.getY(),
							0,
							new Rotation3d(0, 0, simPoseSim2.getRotation().getRadians()));
				},
				secondSimRobot.drive::getFieldRelativeChassisSpeeds, SecondSimRobotOutputs.LOG_ROOT_PREFIX);
			secondSimRobot.shooterSim = new ShooterSim(fuelSim, fuelSimEnabled ? fuelRobotIndex : 0,
					fuelSimEnabled && shooterSimEnabled, secondSimRobot.shooterSimVisualizer);
		} else {
			secondSimRobot.shooterSim = null;
			secondSimRobot.shooterSimVisualizer = null;
		}

		// Fuel Sim
		if (fuelSimEnabled) {
			Runnable intakeFuelCallbackSim2 = secondSimRobot.shooterSim != null ? secondSimRobot.shooterSim::intakeFuel : () -> {};
			registerFuelSimIntake(fuelRobotIndex, secondSimRobot.intake,
					() -> secondSimRobot.extender.getState() == Extender.State.EXTENDED && (secondSimRobot.shooterSim == null || secondSimRobot.shooterSim.canIntake()),
					intakeFuelCallbackSim2);
		}

		// Subsystem Manual Override Ignore Limits Supplier
		secondSimRobot.intake.setIgnoreLimitsSupplier(() 	 -> false);
		secondSimRobot.extender.setIgnoreLimitsSupplier(() -> false);
		secondSimRobot.agitator.setIgnoreLimitsSupplier(() -> false);
		secondSimRobot.transfer.setIgnoreLimitsSupplier(() -> false);
		secondSimRobot.hood.setIgnoreLimitsSupplier(() 		 -> false);
		secondSimRobot.flywheel.setIgnoreLimitsSupplier(() -> false);
		secondSimRobot.hang.setIgnoreLimitsSupplier(() 		 -> false);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------------- Drive Commands --------------------------------------
		/// -------------------------------------------------------------------------------------------
		// Initialize face-target PID controller
		// Rotate so Turret pivot aims at Hub, not Robot center. Same PID constants as DriveCommands.
		secondSimRobot.faceTargetController = new ProfiledPIDController(DriveCommands.getAngleKp(), 0.0, DriveCommands.getAngleKd(),
		new TrapezoidProfile.Constraints(DriveCommands.getAngleMaxVelocity(), DriveCommands.getAngleMaxAcceleration()));
		secondSimRobot.faceTargetController.enableContinuousInput(-Math.PI, Math.PI);

		secondSimRobot.teleopDrive = new TeleopDrive(secondSimRobot.drive, secondSimRobot.driverController,
				() -> secondSimRobot.isRobotCentric, () -> secondSimRobot.isFacingHub, secondSimRobot.faceTargetController,
				() -> kSecondSimRobotRedAlliance,
				SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobot.teleopDrive.setManualOverrideSupplier(() -> false);

		return secondSimRobot;
	} // End buildSecondSimRobot
}
