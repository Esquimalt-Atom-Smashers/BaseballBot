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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSim;
import frc.robot.simulation.HubLightSimDisplay;
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
import frc.robot.subsystems.candle.*;

import frc.robot.simulation.SecondSimRobotBundle;
import frc.robot.simulation.SecondSimRobotOutputs;
import frc.robot.simulation.SimFullFieldExtraRobot;
import frc.robot.simulation.SimFullFieldExtraBehaviourSim;
import frc.robot.simulation.SimSecondRobotHost;
import frc.robot.simulation.SimSecondRobotSession;
import frc.robot.simulation.SimStartingPoseFullFieldSim;
import frc.robot.simulation.SimStartingPoseUtil;


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
	private boolean isHoodEnabled 		= true;
	private boolean isFlywheelEnabled = true;
	private boolean isHangEnabled 		= false;
	private boolean isCandleEnabled 	= true;

	// Simulation Toggle
	private boolean halfFuelOnly 			= false;
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
	private final Hang hang;
	private final CANdle candle;

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
	private boolean chooserRepublishPending = true;
	private boolean ntConnectedLastTick = false;

	// Drive Simulation
	private SwerveDriveSimulation driveSimulation = null;
	/** Last extender state used for dyn4j bumper footprint (SIM only). */
	private boolean driveSimCollisionExtenderExtended = false;

	// Fuel Simulation (robot-ball collision in SIM)
	private final FuelSim fuelSim = new FuelSim();

	// Shooter Simulation and Visualizer (only non-null in SIM)
	private final ShooterSim shooterSim;
	private final ShooterSimVisualizer shooterSimVisualizer;


	// Second sim + starting poses / full-field extras (SIM only; see {@link #simSpawnSim})
	/** USB port for second sim driver gamepad (mirrors port 0 bindings only). */
	private static final int kSecondSimDriverControllerPort = 3;

	private final SimSecondRobotSession simSecondRobotSession = new SimSecondRobotSession();
	private final SimSecondRobotHost simSecondRobotHost = new SimSecondRobotHostImpl();

	/** Null unless {@link Constants.Mode#SIM}: choosers, apply/reset, full-field Maple extras. */
	private SimStartingPoseFullFieldSim simSpawnSim = null;
	/** Null unless {@link Constants.Mode#SIM}: behavior chooser manager for extra robots only. */
	private SimFullFieldExtraBehaviourSim simExtraBehaviorSim = null;
	/** Pool-indexed full-field extras built by {@link #createSimFullFieldExtraRobotBody(int, Pose2d)}. */
	private final SimFullFieldExtraRobot[] extraRobotsByPool = new SimFullFieldExtraRobot[5];

	/** Field Reset SmartDashboard Key */
	private static final String SIM_RESET_SIMULATION_FIELD_KEY = "SimField/ResetSimulationField";

	/** Full duplicate sim robot (driver-only OI); null until enabled from the dashboard in SIM. */
	private SecondSimRobotBundle secondSimBundle = null;


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
							new VisionIOPhotonVision(camera1Name, robotToCamera1),
							new VisionIOPhotonVision(camera2Name, robotToCamera2));
				} else {
					vision = new Vision(drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
				}

				// Subsystems
				intake   = isIntakeEnabled 	 ? new Intake(new IntakeIOSparkMax()) 	  : new Intake(new IntakeIO() {});
				extender = isExtenderEnabled ? new Extender(new ExtenderIOSparkMax()) : new Extender(new ExtenderIO() {});
				agitator = isAgitatorEnabled ? new Agitator(new AgitatorIOSparkMax()) : new Agitator(new AgitatorIO() {});
				transfer = isTransferEnabled ? new Transfer(new TransferIOSparkMax()) : new Transfer(new TransferIO() {});
				turret   = isTurretEnabled 	 ? new Turret(new TurretIOSparkMax()) 	  : new Turret(new TurretIO() {});
				hood     = isHoodEnabled  	 ? new Hood(new HoodIOAxon()) 		  			: new Hood(new HoodIO() {});
				flywheel = isFlywheelEnabled ? new Flywheel(new FlywheelIOTalonFX())  : new Flywheel(new FlywheelIO() {});
				hang 	 	 = isHangEnabled		 ? new Hang(new HangIOSparkMax())  				: new Hang(new HangIO() {});
				candle 	 = isCandleEnabled   ? new CANdle(new CANdleIOLEDs())  			  : new CANdle(new CANdleIO() {});
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

				driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, SimStartingPoseUtil.poseForStem(SimStartingPoseUtil.STEM_RIGHT, false));
				createRobotShape(driveSimulation, 0.0);
				SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
				drive = new Drive(
						new GyroIOSim(driveSimulation.getGyroSimulation()),
						new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0], 0),
						new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1], 1),
						new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2], 2),
						new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3], 3),
						driveSimulation::setSimulationWorldPose);
				// drive.setPose(primaryStartBlue);
				
				// Initialize Vision after Drive (Vision needs Drive reference)
				vision = new Vision(drive,
						new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
						new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose),
						new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, driveSimulation::getSimulatedDriveTrainPose));

				// Subsystems
				intake 	 = new Intake(new IntakeIOSim());
				extender = new Extender(new ExtenderIOSim());
				agitator = new Agitator(new AgitatorIOSim());
				transfer = new Transfer(new TransferIOSim());
				turret 	 = new Turret(new TurretIOSim());
				hood 		 = new Hood(new HoodIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());
				hang 		 = new Hang(new HangIOSim());
				candle   = new CANdle(new CANdleIOSim());

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
				
				simSpawnSim = new SimStartingPoseFullFieldSim(this::createSimFullFieldExtraRobotBody);
				simSpawnSim.init();
				simExtraBehaviorSim = new SimFullFieldExtraBehaviourSim();
				simExtraBehaviorSim.init();
				break;

			// Replayed Robot, disable IO implementations
			default:
				drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
				vision = new Vision(drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});

				// Subsystems
				intake 	 = new Intake(new IntakeIO() {});
				extender = new Extender(new ExtenderIO() {});
				agitator = new Agitator(new AgitatorIO() {});
				transfer = new Transfer(new TransferIO() {});
				turret 	 = new Turret(new TurretIO() {});
				hood 		 = new Hood(new HoodIO() {});
				flywheel = new Flywheel(new FlywheelIO() {});
				hang 		 = new Hang(new HangIO() {});
				candle   = new CANdle(new CANdleIO() {});
				shooterSim = null;
				shooterSimVisualizer = null;
				break;
			}

		// Shooter coordinator and shoot-when-ready command
		shooter = new Shooter(drive, agitator, transfer, turret, hood, flywheel, isHoodEnabled);
		shootWhenReadyCommand = new ShootWhenReadyCommand(agitator, transfer, shooter, drive, () -> drive.getPose());

		shooter.setShootCommandScheduledSupplier(shootWhenReadyCommand::isScheduled);
		shooter.setManualOverrideSupplier(() -> operatorManualOverride);

		safeRetractExtenderCommand =
				SafeRetractExtenderCommand.create(
						shootWhenReadyCommand, flywheel, extender, turret, b -> driverTurretOverride = b);

		// Subsystem Manual Override Ignore Limits and Use SmartDashboard Supplier
		intake.setIgnoreLimitsSupplier(() 	-> operatorManualOverride);
		extender.setIgnoreLimitsSupplier(() -> operatorManualOverride);
		agitator.setIgnoreLimitsSupplier(() -> operatorManualOverride);
		transfer.setIgnoreLimitsSupplier(() -> operatorManualOverride);
		hood.setIgnoreLimitsSupplier(() 		-> operatorManualOverride);
		hood.setUseSmartDashboardTarget(()  -> operatorManualOverride);
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

		// LED Subsystem
		candle.setShootWhenReadySupplier(() -> shootWhenReadyCommand.isScheduled());
		candle.setManualOverrideSupplier(() -> RobotContainer.driverManualOverride || RobotContainer.operatorManualOverride);
		candle.setShooter(shooter);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------------ Logger Dashboard -------------------------------------
		/// -------------------------------------------------------------------------------------------
		// Field view: Robot + Turret so you can see Turret direction in sim
		SmartDashboard.putData("Field", field);
		if (Constants.currentMode == Constants.Mode.SIM) {
			SmartDashboard.putData(SimSecondRobotSession.DASHBOARD_KEY, simSecondRobotSession.getModeChooser());
			SmartDashboard.putBoolean("SimStartingPose/Apply", false);
			SmartDashboard.putBoolean("SimStartingPose/ResetToDefaults", false);
			SmartDashboard.putBoolean("SimFullFieldExtraRobots/Enabled", false);
			SmartDashboard.putBoolean(SIM_RESET_SIMULATION_FIELD_KEY, false);
		}

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
		

    // Configure button bindings
    configureDriverBindings();
    configureOperatorBindings(true); // False to disable operator controls
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
		BooleanSupplier driverTriggerGate = bind.driverTriggerGate();

		drive.setDefaultCommand(teleopDrive);

		// Intake toggle: right bumper = Intaking ↔ Idle, left bumper = Reversing ↔ Idle
		driverController.leftBumper().and(driverTriggerGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.setIdleState(), intake),
				Commands.runOnce(() -> intake.setReversingState(), intake),
				() -> intake.getState() == Intake.State.REVERSING));
		driverController.rightBumper().and(driverTriggerGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.setIdleState(), intake),
				Commands.runOnce(() -> intake.setIntakingState(), intake),
				() -> intake.getState() == Intake.State.INTAKING));

		// Cycle Extender, starts in Retracted, goes to Extended, and then cycles between Partial and Extended. 
		// If in Manual, goes to Extended.
		//driverController.leftTrigger().and(driverTriggerGate).onTrue(Commands.runOnce(() -> {
		//	switch (extender.getState()) {
		//		case EXTENDED ->  extender.setPartialState();
		//		case PARTIAL -> 	extender.setExtendedState();
		//		case RETRACTED -> extender.setExtendedState();
		//		case MANUAL -> 		extender.setExtendedState();
		//		case IDLE ->			extender.setExtendedState();
		//		default -> throw new IllegalArgumentException("Unexpected value: " + extender.getState());
		//	}
//
		//	// Disable Intake when Extender is Retracted or Partial
		//	if (extender.getState() == Extender.State.RETRACTED || extender.getState() == Extender.State.PARTIAL) {
		//		intake.setIdleState();
		//	}
		//}, extender));

		driverController.leftTrigger().and(driverTriggerGate).onTrue(Commands.runOnce(() -> {
			drive.resetPosition(new Pose2d(Constants.Dimensions.FULL_LENGTH.div(2), Constants.Dimensions.FULL_WIDTH.div(2), new Rotation2d(0)));
		}));

		// Set to Retracted, must turn off AutoShoot, and set Turret Target to 0
		driverController.povDown().and(driverTriggerGate).onTrue(safeRetractExtenderCommand);

		// Toggle face-target mode when Y button is pressed
		driverController.y().and(driverTriggerGate).onTrue(Commands.runOnce(() -> {
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
			driverController.b().and(driverTriggerGate).onTrue(
				new ConditionalCommand(
					Commands.runOnce(() -> hang.setLevel1State(), hang),
					new ConditionalCommand(
						Commands.runOnce(() -> hang.setIdleState(), hang),
						Commands.runOnce(() -> hang.setLevel1State(), hang),
						() -> hang.getState() == Hang.State.LEVEL_1),
					() -> hangHoldModeEnabled));
			driverController.b().and(driverTriggerGate).onFalse(
				new ConditionalCommand(
					Commands.runOnce(() -> hang.setIdleState(), hang),
					new InstantCommand(),
					() -> hangHoldModeEnabled));
			// Before Endgame, X toggles Stored / Idle. 
			// Endgame:        X onTrue sets hangHoldModeEnabled (see B); Hanging state while held, Idle on release
			driverController.x().and(driverTriggerGate).onTrue(
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
			driverController.x().and(driverTriggerGate).whileTrue(
				new ConditionalCommand(
					Commands.startEnd(
						() -> hang.setHangingState(),
						() -> hang.setIdleState(),
						hang),
					new InstantCommand(),
					() -> isHangDriverEndgamePeriod()));
		}

		// Shoot toggle: on = schedule ShootWhenReadyCommand, set Flywheel to Charging if Idle; off = cancel (command end() sets Transfer and Agitator to Idle)
		driverController.a().and(driverTriggerGate).onTrue(Commands.runOnce(() -> {
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
		driverController.start().and(driverTriggerGate).onTrue(
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

		// Operator Control Gate
		Trigger operatorControlGate = new Trigger(this::isOperatorControlsEnabled);

		// Intake Manual Voltage Control
		// Raise Intake voltage
		operatorController.povLeft().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.stepVoltage(IntakeConstants.kStepVolts), intake),
				new InstantCommand(),
				() -> intake != null
			)
		);
		// Lower Intake voltage
		operatorController.povRight().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.stepVoltage(-IntakeConstants.kStepVolts), intake),
				new InstantCommand(),
				() -> intake != null
			)
		);

		// Extender Manual Position Control
		// Raise Extender Position
		operatorController.leftTrigger().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					extender.stepPositionRad(ExtenderConstants.kStepRadUp);
				}),
				new InstantCommand(),
				() -> extender != null)
		);
		// Lower Extender Position
		operatorController.rightTrigger().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					extender.stepPositionRad(-ExtenderConstants.kStepRadDown);
				}),
				new InstantCommand(),
				() -> extender != null)
		);

		// Hang Manual Position Control
		// Raise Hang (Extend)
		operatorController.b().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> hang.stepPositionMeters(HangConstants.kStepMeters), hang),
				new InstantCommand(),
				() -> hang != null
			)
		);
		// Lower Hang (Retract)
		operatorController.x().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> hang.stepPositionMeters(-HangConstants.kStepMeters), hang),
				new InstantCommand(),
				() -> hang != null
			)
		);


		// ------------------------------------------ Operator Manual Override ------------------------------------------
		// If Manual Override is false, become true. 
		// If true, reset encoder positions and then become false.
		operatorController.back().and(operatorControlGate).onTrue(
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
		operatorController.a().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> agitator.stepVoltage(-AgitatorConstants.kStepVolts), hood),
				new InstantCommand(),
				() -> (operatorManualOverride && agitator != null)
			)
		);

		// Transfer Manual Voltage Control
		// Raise Transfer voltage
		operatorController.leftBumper().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> transfer.stepVoltage(TransferConstants.kStepVolts), transfer),
				new InstantCommand(),
				() -> (operatorManualOverride && transfer != null)
			)
		);
		// Lower Transfer voltage
		operatorController.rightBumper().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> transfer.stepVoltage(-TransferConstants.kStepVolts), transfer),
				new InstantCommand(),
				() -> (operatorManualOverride && transfer != null)
			)
		);
		
		// Turret Manual Position Control
		// Step Turret position up
		operatorController.leftStick().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> turret.stepPositionRad(TurretConstants.kStepRad), turret),
				new InstantCommand(),
				() -> (operatorManualOverride && turret != null)
			)
		);
		// Step Turret position down
		operatorController.rightStick().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> turret.stepPositionRad(-TurretConstants.kStepRad), turret),
				new InstantCommand(),
				() -> (operatorManualOverride && turret != null)
			)
		);
		// Reset Extender and Turret Encoder, and Idle all subsystems.
		operatorController.start().and(operatorControlGate).onTrue(
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
		operatorController.povUp().and(operatorControlGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> flywheel.stepVelocityRadPerSec(FlywheelConstants.kStepRadPerSec), flywheel),
				new InstantCommand(),
				() -> (operatorManualOverride && flywheel != null)
			)
		);
		// Lower Flywheel RPM
		operatorController.povDown().and(operatorControlGate).onTrue(
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
			boolean bindPrimaryOnlyExtras,
			BooleanSupplier driverTriggerGate) {
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
				true,
				() -> true);
	} // End createPrimaryDriverBindParams

	/** Create Driver bindings for second sim robot. */
	private DriverBindParams createSecondSimDriverBindParams() {
		SecondSimRobotBundle secondSimRobotBundle = secondSimBundle;
		Runnable resetGyro =
				() -> secondSimRobotBundle.drive.setPose(secondSimRobotBundle.driveSimulation.getSimulatedDriveTrainPose());
		return new DriverBindParams(
				secondSimRobotBundle.driverController, 
				secondSimRobotBundle.drive, secondSimRobotBundle.teleopDrive,
				secondSimRobotBundle.intake, secondSimRobotBundle.extender, secondSimRobotBundle.flywheel, secondSimRobotBundle.hang,
				secondSimRobotBundle.shootWhenReady, secondSimRobotBundle.faceTargetController,
				secondSimRobotBundle.safeRetractExtenderCommand,
				resetGyro,
				() -> secondSimRobotBundle.isFacingHub, enabled -> secondSimRobotBundle.isFacingHub = enabled,
				() -> secondSimRobotBundle.isRobotCentric, enabled -> secondSimRobotBundle.isRobotCentric = enabled,
				() -> true,
				false,
				() -> simSecondRobotSession.isDriveEnabledFromDashboard());
	} // End createSecondSimDriverBindParams

	/** Disable operator bindings when full-field extras are enabled in SIM (port 1 is reassigned to Blue-2). */
	private boolean isOperatorControlsEnabled() {
		return !(Constants.currentMode == Constants.Mode.SIM
				&& simSpawnSim != null
				&& simSpawnSim.extrasEnabled());
	} // End isOperatorControlsEnabled

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
		NamedCommands.registerCommand("Set Shooter Target Hub", Commands.runOnce(() -> ShooterCommands.clearShooterTargetOverride(drive)));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Left", Commands.runOnce(() -> ShooterCommands.setPassingSpotLeft(drive)));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Center", Commands.runOnce(() -> ShooterCommands.setPassingSpotCenter(drive)));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Right", Commands.runOnce(() -> ShooterCommands.setPassingSpotRight(drive)));
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

	/**
	 * Re-publishes chooser selected values once on NT connect (and again on reconnect) so dashboard chooser widgets
	 * bind reliably without manual topic re-confirm.
	 */
	public void republishDashboardChoosersOnNtConnect() {
		NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
		boolean ntConnected = ntInstance.isConnected();
		if (ntConnected && !ntConnectedLastTick) {
			chooserRepublishPending = true;
		}
		ntConnectedLastTick = ntConnected;
		if (!ntConnected || !chooserRepublishPending) {
			return;
		}

		republishChooserSelectedEntry("Auto Choices");
		if (simSpawnSim != null) {
			simSpawnSim.republishSelectedChoices();
		}
		if (simExtraBehaviorSim != null) {
			simExtraBehaviorSim.republishSelectedChoices();
		}
		chooserRepublishPending = false;
	} // End republishDashboardChoosersOnNtConnect

	/**
	 * Writes chooser {@code selected} back to itself for one SmartDashboard chooser root.
	 *
	 * <p>If {@code selected} is blank, falls back to {@code active} when present.
	 */
	private static void republishChooserSelectedEntry(String chooserRootKey) {
		NetworkTable chooserTable = NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(chooserRootKey);
		String selected = chooserTable.getEntry("selected").getString("");
		if (selected == null || selected.isEmpty()) {
			selected = chooserTable.getEntry("active").getString("");
		}
		if (selected != null && !selected.isEmpty()) {
			chooserTable.getEntry("selected").setString(selected);
		}
	} // End republishChooserSelectedEntry

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

		if (secondSimBundle != null) {
			CommandScheduler.getInstance().cancel(secondSimBundle.shootWhenReady);
			if (secondSimBundle.intake != null)	 secondSimBundle.intake.setIdleState();
			if (secondSimBundle.extender != null) secondSimBundle.extender.setIdleState();
			if (secondSimBundle.agitator != null) secondSimBundle.agitator.setIdleState();
			if (secondSimBundle.transfer != null) secondSimBundle.transfer.setIdleState();
			if (secondSimBundle.flywheel != null) secondSimBundle.flywheel.setState(Flywheel.State.IDLE);
			if (secondSimBundle.hang != null) 		secondSimBundle.hang.setIdleState();
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
	/**
	 * SIM only: applies chooser starting poses (same path as Elastic {@code SimStartingPose/Apply}), resets fuel when
	 * {@link #fuelSimEnabled}. Called from {@link Robot#disabledInit} and from Elastic via {@value #SIM_RESET_SIMULATION_FIELD_KEY}.
	 */
	public void resetSimulationField() {
		if (Constants.currentMode != Constants.Mode.SIM) {
			return;
		}
		runSimulationFieldReset();
	} // End resetSimulationField

	/**
	 * Pulses {@code SimStartingPose/Apply} and runs the same {@link SimStartingPoseFullFieldSim#pollApplyButton} logic
	 * as the Elastic Apply control, then {@link FuelSim#resetFuel()} if fuel sim is enabled.
	 */
	private void runSimulationFieldReset() {
		if (simSpawnSim == null) {
			return;
		}
		SmartDashboard.putBoolean("SimStartingPose/Apply", true);
		pollSimStartingPosesApplyButton();
		if (fuelSimEnabled) {
			fuelSim.resetFuel();
		}
	} // End runSimulationFieldReset

	/** If {@value #SIM_RESET_SIMULATION_FIELD_KEY} is true, clears it and runs {@link #runSimulationFieldReset()}. */
	private void pollResetSimulationFieldDashboardButton() {
		if (!SmartDashboard.getBoolean(SIM_RESET_SIMULATION_FIELD_KEY, false)) {
			return;
		}
		SmartDashboard.putBoolean(SIM_RESET_SIMULATION_FIELD_KEY, false);
		runSimulationFieldReset();
	} // End pollResetSimulationFieldDashboardButton

	/** Applies primary robot pose from starting-pose choosers to Maple sim and odometry. */
	private void applyPrimarySimStartingPoseFromChooser(Pose2d appliedPose) {
		driveSimulation.setSimulationWorldPose(appliedPose);
		drive.setPose(appliedPose);
	} // End applyPrimarySimStartingPoseFromChooser

	/** Applies second-sim robot pose from starting-pose choosers when that bundle exists. */
	private void applySecondSimStartingPoseFromChooser(Pose2d secondSimAppliedPose) {
		if (secondSimBundle != null) {
			secondSimBundle.driveSimulation.setSimulationWorldPose(secondSimAppliedPose);
			secondSimBundle.drive.setPose(secondSimAppliedPose);
		}
	} // End applySecondSimStartingPoseFromChooser

	/** Consumes Elastic {@code SimStartingPose/Apply} when true and teleports robots to chooser poses. */
	private void pollSimStartingPosesApplyButton() {
		simSpawnSim.pollApplyButton(
				true,
				simSecondRobotSession.isDriveEnabledFromDashboard(),
				simSecondRobotSession.isRedAlliance(),
				this::applyPrimarySimStartingPoseFromChooser,
				this::applySecondSimStartingPoseFromChooser);
	} // End pollSimStartingPosesApplyButton

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
		SmartDashboard.putData(
				Commands.runOnce(() -> fuelSim.resetFuel()).withName("Reset Fuel").ignoringDisable(true));
  } // End configureFuelSim

	/**
	 * Register a robot for collision with fuel using the default carried-fuel row layout for additional slots.
	 *
	 * @param driveForFuel drive used for field-relative chassis speeds supplied to {@link FuelSim}
	 * @param additionalRobotSlot when true, registers as an extra slot with default carried-fuel rows
	 * @param mapleDriveSimulation Maple dyn4j chassis for that robot; FuelSim applies opposing impulses when fuel is
	 *     depenetrated (must be the same instance used for pose/speed suppliers’ sim)
	 */
	private int registerFuelSimRobotBody(Drive driveForFuel, boolean additionalRobotSlot, SwerveDriveSimulation mapleDriveSimulation) {
		return registerFuelSimRobotBody(
				driveForFuel,
				additionalRobotSlot,
				mapleDriveSimulation,
				FuelSim.kCarriedFuelRowsBackToFrontDefault);
	} // End registerFuelSimRobotBody

	/**
	 * Register a robot for collision with fuel. When {@code additionalRobotSlot} is true, {@code
	 * carriedFuelRowsBackToFront} is forwarded to {@link FuelSim} to configure carried-fuel stack row order for that
	 * slot.
	 *
	 * @param driveForFuel drive used for field-relative chassis speeds supplied to {@link FuelSim}
	 * @param additionalRobotSlot when true, registers as an extra slot with the given carried-fuel row layout
	 * @param mapleDriveSimulation Maple dyn4j chassis for that robot; FuelSim applies opposing impulses when fuel is
	 *     depenetrated (must be the same instance used for pose/speed suppliers’ sim)
	 * @param carriedFuelRowsBackToFront row-order value for additional slots; ignored when {@code additionalRobotSlot}
	 *     is false
	 */
	private int registerFuelSimRobotBody(
			Drive driveForFuel,
			boolean additionalRobotSlot,
			SwerveDriveSimulation mapleDriveSimulation,
			int carriedFuelRowsBackToFront) {
		double robotWidthMeters = Constants.Dimensions.FULL_WIDTH.in(Meters);
		double robotLengthMeters = Constants.Dimensions.FULL_LENGTH.in(Meters);
		double bumperHeightMeters = Constants.Dimensions.BUMPER_HEIGHT.in(Meters);

		if (additionalRobotSlot) {
			return fuelSim.addRegisteredRobot(
					robotWidthMeters,
					robotLengthMeters,
					bumperHeightMeters,
					mapleDriveSimulation::getSimulatedDriveTrainPose,
					driveForFuel::getFieldRelativeChassisSpeeds,
					mapleDriveSimulation,
					carriedFuelRowsBackToFront);
		}
		fuelSim.registerRobot(
				robotWidthMeters,
				robotLengthMeters,
				bumperHeightMeters,
				mapleDriveSimulation::getSimulatedDriveTrainPose,
				driveForFuel::getFieldRelativeChassisSpeeds, mapleDriveSimulation);
		return 0;
	} // End registerFuelSimRobotBody

	/**
	 * Registers the standard robot-frame front intake box on {@link FuelSim} for the given registered robot index.
	 *
	 * @param fuelRobotIndex registered robot index on {@link FuelSim} for this intake region
	 * @param ableToIntake when false, fuel in the box is not vacuumed this tick
	 * @param intakeCallback runs once each time a fuel is removed by this intake
	 */
	private void registerFuelSimFrontIntakeBox(
			int fuelRobotIndex, BooleanSupplier ableToIntake, Runnable intakeCallback) {
		double robotWidthMeters = Constants.Dimensions.FULL_WIDTH.in(Meters);
		double robotLengthMeters = Constants.Dimensions.FULL_LENGTH.in(Meters);
		// Intake: 10.5" beyond front of frame (+X), full width minus 2" on each side
		double intakeExtendMeters = 10.5 * 0.0254;
		double intakeInsetMeters = 2.0 * 0.0254;
		fuelSim.registerIntake(
				fuelRobotIndex,
				robotLengthMeters / 2,
				robotLengthMeters / 2 + intakeExtendMeters,
				-robotWidthMeters / 2 + intakeInsetMeters,
				robotWidthMeters / 2 - intakeInsetMeters,
				ableToIntake,
				intakeCallback);
	} // End registerFuelSimFrontIntakeBox

	/**
	 * Registers the front intake box for a sim drive that uses a real {@link Intake} subsystem predicate.
	 *
	 * @param fuelRobotIndex registered robot index on {@link FuelSim}
	 * @param intakeForFuel intake subsystem whose state is combined with {@code ableToIntake}
	 * @param ableToIntake extra gate; both this and intaking state must be true to vacuum
	 * @param intakeCallback runs once each time a fuel is removed by this intake
	 */
	private void registerFuelSimIntake(
			int fuelRobotIndex, Intake intakeForFuel, BooleanSupplier ableToIntake, Runnable intakeCallback) {
		registerFuelSimFrontIntakeBox(
				fuelRobotIndex,
				() -> ableToIntake.getAsBoolean() && intakeForFuel.getState() == Intake.State.INTAKING,
				intakeCallback);
	} // End registerFuelSimIntake

	/**
	 * Registers a front intake box that vacuums field fuel while carried count stays below
	 * {@link FuelSim#getCarriedFuelMaxForRobotIndex(int)} for {@code fuelRobotIndex}; intake callback is a no-op.
	 *
	 * @param fuelRobotIndex registered robot index on {@link FuelSim}
	 */
	private void registerFuelSimExtraRobotIntake(int fuelRobotIndex) {
		double robotWidthMeters = 0.8382;
		double robotLengthMeters = 0.83312;
		double intakeExtendMeters = 3.0 * 0.0254;
		double intakeInsetMeters = 0.20955;
		fuelSim.registerIntake(
				fuelRobotIndex,
				robotLengthMeters / 2.0,
				robotLengthMeters / 2.0 + intakeExtendMeters,
				-robotWidthMeters / 2 + intakeInsetMeters,
				robotWidthMeters / 2 - intakeInsetMeters,
				() -> fuelSim.getCarriedFuelCount(fuelRobotIndex) < fuelSim.getCarriedFuelMaxForRobotIndex(fuelRobotIndex),
				() -> {});
	} // End registerFuelSimExtraRobotIntake

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

		pollResetSimulationFieldDashboardButton();

		simExtraBehaviorSim.pollResetToDefaults(true);
		simSpawnSim.pollResetToDefaults(true, SimSecondRobotSession::forceModeDisableViaNt);
		simSecondRobotSession.poll(
				simSecondRobotHost,
				() -> simSpawnSim.selectedStem(SimStartingPoseFullFieldSim.ROLE_SECOND_SIM));
		simSpawnSim.pollFullFieldExtrasDashboard(
				true,
				simSecondRobotSession.isDriveEnabledFromDashboard(),
				simSecondRobotSession.isRedAlliance());
		simSpawnSim.pollDuplicateSwap(
				true,
				simSecondRobotSession.isDriveEnabledFromDashboard(),
				simSecondRobotSession.isRedAlliance());
		pollSimStartingPosesApplyButton();
		updateFullFieldExtraRobotBehaviors();

		boolean extenderExtendedForCollision = extender.getState() == Extender.State.EXTENDED;
		if (extenderExtendedForCollision != driveSimCollisionExtenderExtended) {
			driveSimCollisionExtenderExtended = extenderExtendedForCollision;
			createRobotShape(
					driveSimulation,
					extenderExtendedForCollision ? Constants.Dimensions.kExtensionPastBumpersMeters : 0.0);
		}

		if (secondSimBundle != null) {
			boolean secondSimExtenderExtendedForCollision =
					secondSimBundle.extender.getState() == Extender.State.EXTENDED;
			if (secondSimExtenderExtendedForCollision != secondSimBundle.driveSimCollisionExtenderExtended) {
				secondSimBundle.driveSimCollisionExtenderExtended = secondSimExtenderExtendedForCollision;
				createRobotShape(
						secondSimBundle.driveSimulation,
						secondSimExtenderExtendedForCollision ? Constants.Dimensions.kExtensionPastBumpersMeters : 0.0);
			}
		}

		SimulatedArena.getInstance().simulationPeriodic();

		// Hub Light Simulation
		HubLightSimDisplay.update();

		// Robot pose for visualization
		Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
		Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);
		
		if (secondSimBundle != null && simSecondRobotSession.isDriveEnabledFromDashboard()) {
			Pose2d secondSimRobotPose = secondSimBundle.driveSimulation.getSimulatedDriveTrainPose();
			Logger.recordOutput(SecondSimRobotOutputs.fieldSimulationRobotPositionKey(simSecondRobotSession.isRedAlliance()), secondSimRobotPose);
			Logger.recordOutput(SecondSimRobotOutputs.LOG_ROOT_PREFIX + "TeleopDrive/IsFacingHub", secondSimBundle.isFacingHub);
			Logger.recordOutput(SecondSimRobotOutputs.LOG_ROOT_PREFIX + "TeleopDrive/IsRobotCentric", secondSimBundle.isRobotCentric);
			field.getObject("robotPoseSim2").setPose(secondSimRobotPose);
		}

		Logger.recordOutput("ComponentPoses/Final", buildComponentPoses(turret, extender, hang));
		if (secondSimBundle != null && simSecondRobotSession.isDriveEnabledFromDashboard()) {
			Logger.recordOutput(
					SecondSimRobotOutputs.LOG_ROOT_PREFIX + "ComponentPoses/Final",
					buildComponentPoses(secondSimBundle.turret, secondSimBundle.extender, secondSimBundle.hang));
		}

		// Update field view
		field.setRobotPose(robotPose);

		// Shooter sim: launch fuel only when Shooter is ready and shoot command is active
		if (shooterSim != null) {
			shooterSim.update(shooter, shooter::isShootCommandActive, turret, hood, flywheel);
		}
		if (secondSimBundle != null && secondSimBundle.shooterSim != null) {
			secondSimBundle.shooterSim.update(secondSimBundle.shooter, secondSimBundle.shooter::isShootCommandActive, secondSimBundle.turret, secondSimBundle.hood, secondSimBundle.flywheel);
		}
		if (shooterSimVisualizer != null) {
			double hoodAngleRad = isHoodEnabled ? hood.getAngleRad() : HoodConstants.kDisabledAngleRad;
			double flywheelSurfaceMps = flywheel.getVelocityRadPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
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
		if (secondSimBundle != null && secondSimBundle.shooterSimVisualizer != null) {
			double secondSimHoodAngleRad = isHoodEnabled ? secondSimBundle.hood.getAngleRad() : HoodConstants.kDisabledAngleRad;
			double secondSimFlywheelSurfaceMps =
					secondSimBundle.flywheel.getVelocityRadPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
			double secondSimBallExitVelMps = secondSimFlywheelSurfaceMps
					* ShooterConstants.kFlywheelSurfaceDivider
					* ShooterConstants.kSimFlywheelToFuelExitVelocityEfficiency;
			secondSimBundle.shooterSimVisualizer.updateFuel(
					edu.wpi.first.units.Units.MetersPerSecond.of(secondSimBallExitVelMps),
					edu.wpi.first.units.Units.Radians.of(secondSimHoodAngleRad),
					edu.wpi.first.units.Units.Radians.of(secondSimBundle.turret.getRobotFramePosition().getRadians()));
			secondSimBundle.shooterSimVisualizer.update3dPose(
					edu.wpi.first.units.Units.Radians.of(secondSimBundle.turret.getRobotFramePosition().getRadians()),
					edu.wpi.first.units.Units.Radians.of(secondSimHoodAngleRad));
		}

		// Fuel sim (robot-ball collision)
		if (fuelSimEnabled) {
			fuelSim.updateSim();
		}

		// Log balls in robot and hub scores (FuelSim)
		if (shooterSim != null) {
			Logger.recordOutput("FuelSim/BallsInRobot", shooterSim.getFuelStored());
		}
		if (secondSimBundle != null && simSecondRobotSession.isDriveEnabledFromDashboard() && secondSimBundle.shooterSim != null) {
			Logger.recordOutput(SecondSimRobotOutputs.LOG_ROOT_PREFIX + "FuelSim/BallsInRobot", secondSimBundle.shooterSim.getFuelStored());
		}
		if (fuelSimEnabled) {
			Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
			Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
		}

		// Second Sim Robot and Full Field Extras State
		Logger.recordOutput("Sim/SecondRobotDashboardEnabled", secondSimBundle != null && simSecondRobotSession.isDriveEnabledFromDashboard());
		Logger.recordOutput("Sim/FullFieldExtraRobotsEnabled", simSpawnSim.extrasEnabled());

		simSpawnSim.logExtraPosesIfEnabled(
				simSecondRobotSession.isDriveEnabledFromDashboard(),
				simSecondRobotSession.isRedAlliance());
	} // End updateSimulation

	/** Updates extra robot behaviors. */
	private void updateFullFieldExtraRobotBehaviors() {
		if (simSpawnSim == null || !simSpawnSim.extrasEnabled()) {
			return;
		}

		Pose2d primaryPose = driveSimulation.getSimulatedDriveTrainPose();
		Pose2d secondSimPose = null;
		if (secondSimBundle != null && simSecondRobotSession.isDriveEnabledFromDashboard()) {
			secondSimPose = secondSimBundle.driveSimulation.getSimulatedDriveTrainPose();
		}

		simExtraBehaviorSim.updateExtraRobotBehaviors(
				extraRobotsByPool,
				simSpawnSim.extrasEnabled(),
				simSecondRobotSession.isDriveEnabledFromDashboard(),
				simSecondRobotSession.isRedAlliance(),
				DriverStation.isTeleopEnabled(),
				primaryPose,
				secondSimPose,
				fuelSimEnabled ? fuelSim : null,
				fuelSimEnabled);
	} // End updateFullFieldExtraRobotBehaviors

	/// ------------------------------------------------ Second Sim Robot -----------------------------------------------
	private SecondSimRobotBundle assembleSecondSimRobotBundle() {
		SecondSimRobotBundle secondSimRobotBundle = new SecondSimRobotBundle();
		secondSimRobotBundle.driverController = new CommandXboxController(kSecondSimDriverControllerPort);

		Pose2d secondStartBlue = SimStartingPoseUtil.poseForStem(SimStartingPoseUtil.STEM_CENTER, false);
		secondSimRobotBundle.driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, secondStartBlue);
		createRobotShape(secondSimRobotBundle.driveSimulation, 0.0);
		SimulatedArena.getInstance().addDriveTrainSimulation(secondSimRobotBundle.driveSimulation);
		secondSimRobotBundle.drive = new Drive(
				new GyroIOSim(secondSimRobotBundle.driveSimulation.getGyroSimulation()),
				new ModuleIOSimMapleDirect(TunerConstants.FrontLeft, secondSimRobotBundle.driveSimulation.getModules()[0], 0),
				new ModuleIOSimMapleDirect(TunerConstants.FrontRight, secondSimRobotBundle.driveSimulation.getModules()[1], 1),
				new ModuleIOSimMapleDirect(TunerConstants.BackLeft, secondSimRobotBundle.driveSimulation.getModules()[2], 2),
				new ModuleIOSimMapleDirect(TunerConstants.BackRight, secondSimRobotBundle.driveSimulation.getModules()[3], 3),
				secondSimRobotBundle.driveSimulation::setSimulationWorldPose,
				false,
				Drive.Telemetry.forLogRoot(SecondSimRobotOutputs.LOG_ROOT_PREFIX),
				secondSimRobotBundle.driveSimulation::getSimulatedDriveTrainPose);
		secondSimRobotBundle.drive.setPose(secondStartBlue);

		secondSimRobotBundle.intake 	= new Intake(new IntakeIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobotBundle.extender = new Extender(new ExtenderIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobotBundle.agitator = new Agitator(new AgitatorIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobotBundle.transfer = new Transfer(new TransferIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobotBundle.turret 	= new Turret(new TurretIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobotBundle.hood 		= new Hood(new HoodIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobotBundle.flywheel = new Flywheel(new FlywheelIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX, SecondSimRobotOutputs.smartDashboardPrefix("Flywheel"));
		secondSimRobotBundle.hang 		= new Hang(new HangIOSim(), SecondSimRobotOutputs.LOG_ROOT_PREFIX);

		int fuelRobotIndex = 0;
		if (fuelSimEnabled) {
			fuelRobotIndex = registerFuelSimRobotBody(secondSimRobotBundle.drive, true, secondSimRobotBundle.driveSimulation);
		}

		if (shooterSimEnabled) {
			secondSimRobotBundle.shooterSimVisualizer = new ShooterSimVisualizer(() -> {
					Pose2d secondSimDrivePose = secondSimRobotBundle.driveSimulation.getSimulatedDriveTrainPose();
					return new Pose3d(
							secondSimDrivePose.getX(),
							secondSimDrivePose.getY(),
							0,
							new Rotation3d(0, 0, secondSimDrivePose.getRotation().getRadians()));
				},
				secondSimRobotBundle.drive::getFieldRelativeChassisSpeeds, SecondSimRobotOutputs.LOG_ROOT_PREFIX);
			secondSimRobotBundle.shooterSim = new ShooterSim(fuelSim, fuelSimEnabled ? fuelRobotIndex : 0,
					fuelSimEnabled && shooterSimEnabled, secondSimRobotBundle.shooterSimVisualizer);
		} else {
			secondSimRobotBundle.shooterSim = null;
			secondSimRobotBundle.shooterSimVisualizer = null;
		}

		if (fuelSimEnabled) {
			Runnable secondSimIntakeFuelCallback =
					secondSimRobotBundle.shooterSim != null ? secondSimRobotBundle.shooterSim::intakeFuel : () -> {};
			registerFuelSimIntake(fuelRobotIndex, secondSimRobotBundle.intake,
					() -> secondSimRobotBundle.extender.getState() == Extender.State.EXTENDED
							&& (secondSimRobotBundle.shooterSim == null || secondSimRobotBundle.shooterSim.canIntake()),
					secondSimIntakeFuelCallback);
		}

		secondSimRobotBundle.intake.setIgnoreLimitsSupplier(() 	 -> false);
		secondSimRobotBundle.extender.setIgnoreLimitsSupplier(() -> false);
		secondSimRobotBundle.agitator.setIgnoreLimitsSupplier(() -> false);
		secondSimRobotBundle.transfer.setIgnoreLimitsSupplier(() -> false);
		secondSimRobotBundle.hood.setIgnoreLimitsSupplier(() 		 -> false);
		secondSimRobotBundle.hood.setUseSmartDashboardTarget(()  -> false);
		secondSimRobotBundle.flywheel.setIgnoreLimitsSupplier(() -> false);
		secondSimRobotBundle.hang.setIgnoreLimitsSupplier(() 		 -> false);

		secondSimRobotBundle.faceTargetController = new ProfiledPIDController(DriveCommands.getAngleKp(), 0.0, DriveCommands.getAngleKd(),
		new TrapezoidProfile.Constraints(DriveCommands.getAngleMaxVelocity(), DriveCommands.getAngleMaxAcceleration()));
		secondSimRobotBundle.faceTargetController.enableContinuousInput(-Math.PI, Math.PI);

		secondSimRobotBundle.teleopDrive = new TeleopDrive(secondSimRobotBundle.drive, secondSimRobotBundle.driverController,
				() -> secondSimRobotBundle.isRobotCentric, () -> secondSimRobotBundle.isFacingHub, secondSimRobotBundle.faceTargetController,
				() -> simSecondRobotSession.isRedAlliance(),
				SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobotBundle.teleopDrive.setManualOverrideSupplier(() -> false);

		return secondSimRobotBundle;
	} // End assembleSecondSimRobotBundle

	private void finishSecondSimRobotSetup(SecondSimRobotBundle secondSimRobotBundle) {
		secondSimRobotBundle.shooter = new Shooter(secondSimRobotBundle.drive, secondSimRobotBundle.agitator, secondSimRobotBundle.transfer,
				secondSimRobotBundle.turret, secondSimRobotBundle.hood, secondSimRobotBundle.flywheel, isHoodEnabled, SecondSimRobotOutputs.LOG_ROOT_PREFIX);
		secondSimRobotBundle.shootWhenReady =
				new ShootWhenReadyCommand(secondSimRobotBundle.agitator, secondSimRobotBundle.transfer, secondSimRobotBundle.shooter, secondSimRobotBundle.drive, secondSimRobotBundle.drive::getPose);

		secondSimRobotBundle.shooter.setShootCommandScheduledSupplier(secondSimRobotBundle.shootWhenReady::isScheduled);
		secondSimRobotBundle.shooter.setManualOverrideSupplier(() -> operatorManualOverride);
		secondSimRobotBundle.shooter.setIsRedAllianceSupplier(() -> simSecondRobotSession.isRedAlliance());
		ShooterCommands.registerTargetAllianceSupplier(secondSimRobotBundle.drive, () -> simSecondRobotSession.isRedAlliance());

		secondSimRobotBundle.safeRetractExtenderCommand =
				SafeRetractExtenderCommand.create(
						secondSimRobotBundle.shootWhenReady, secondSimRobotBundle.flywheel, secondSimRobotBundle.extender, secondSimRobotBundle.turret,
						driverTurretOverrideEnabled -> secondSimRobotBundle.driverTurretOverride = driverTurretOverrideEnabled);

		secondSimRobotBundle.turret.setManualOverrideSupplier(() -> operatorManualOverride || secondSimRobotBundle.driverTurretOverride);
		secondSimRobotBundle.turret.setDrive(secondSimRobotBundle.drive);
		secondSimRobotBundle.turret.setAimAtTargetSupplier(() -> secondSimRobotBundle.shootWhenReady.isScheduled());

		configureDriverBindings(createSecondSimDriverBindParams());
	} // End finishSecondSimRobotSetup

	private SimFullFieldExtraRobot createSimFullFieldExtraRobotBody(int poolIdx, Pose2d park) {
		SimFullFieldExtraRobot fullFieldExtraRobot = new SimFullFieldExtraRobot();
		fullFieldExtraRobot.driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, park);
		createRobotShape(fullFieldExtraRobot.driveSimulation, 0.0);
		SimulatedArena.getInstance().addDriveTrainSimulation(fullFieldExtraRobot.driveSimulation);
		String logRoot = "SimFullFieldExtra/" + poolIdx + "/";
		fullFieldExtraRobot.drive = new Drive(
				new GyroIOSim(fullFieldExtraRobot.driveSimulation.getGyroSimulation()),
				new ModuleIOSimMapleDirect(TunerConstants.FrontLeft, fullFieldExtraRobot.driveSimulation.getModules()[0], 0),
				new ModuleIOSimMapleDirect(TunerConstants.FrontRight, fullFieldExtraRobot.driveSimulation.getModules()[1], 1),
				new ModuleIOSimMapleDirect(TunerConstants.BackLeft, fullFieldExtraRobot.driveSimulation.getModules()[2], 2),
				new ModuleIOSimMapleDirect(TunerConstants.BackRight, fullFieldExtraRobot.driveSimulation.getModules()[3], 3),
				fullFieldExtraRobot.driveSimulation::setSimulationWorldPose,
				false,
				Drive.Telemetry.forLogRoot(logRoot),
				fullFieldExtraRobot.driveSimulation::getSimulatedDriveTrainPose);
		fullFieldExtraRobot.drive.setDefaultCommand(Commands.run(fullFieldExtraRobot.drive::stop, fullFieldExtraRobot.drive));
		if (fuelSimEnabled) {
			int fuelIdx = registerFuelSimRobotBody(
					fullFieldExtraRobot.drive,
					true,
					fullFieldExtraRobot.driveSimulation,
					FuelSim.kCarriedFuelRowsBackToFrontSimExtra);
			fullFieldExtraRobot.fuelRobotIndex = fuelIdx;
			fuelSim.setCarriedFuelCount(fuelIdx, ShooterSim.kInitialFuelStored);
			fuelSim.registerShooterFuelReset(() -> fuelSim.setCarriedFuelCount(fuelIdx, ShooterSim.kInitialFuelStored));
			registerFuelSimExtraRobotIntake(fuelIdx);
		}
		if (poolIdx >= 0 && poolIdx < extraRobotsByPool.length) {
			extraRobotsByPool[poolIdx] = fullFieldExtraRobot;
		}
		return fullFieldExtraRobot;
	} // End createSimFullFieldExtraRobotBody

	private final class SimSecondRobotHostImpl implements SimSecondRobotHost {

		@Override
		public SecondSimRobotBundle getSecondSimBundle() {
			return secondSimBundle;
		} // End getSecondSimBundle

		@Override
		public void setSecondSimBundle(SecondSimRobotBundle bundle) {
			secondSimBundle = bundle;
		} // End setSecondSimBundle

		@Override
		public SecondSimRobotBundle buildSecondSimBundle() {
			return assembleSecondSimRobotBundle();
		} // End buildSecondSimBundle

		@Override
		public void finishSecondSimSetup(SecondSimRobotBundle bundle) {
			finishSecondSimRobotSetup(bundle);
		} // End finishSecondSimSetup

		@Override
		public void applySecondSimPoseFromStem(String stem, boolean redAlliance) {
			if (secondSimBundle == null) {
				return;
			}
			Pose2d appliedPose = SimStartingPoseUtil.poseForStem(stem, redAlliance);
			secondSimBundle.driveSimulation.setSimulationWorldPose(appliedPose);
			secondSimBundle.drive.setPose(appliedPose);
		} // End applySecondSimPoseFromStem

		@Override
		public void parkSecondSimOffField() {
			if (secondSimBundle == null) {
				return;
			}
			Pose2d parkPose = new Pose2d(-8.5, -1.5, new Rotation2d());
			secondSimBundle.driveSimulation.setSimulationWorldPose(parkPose);
			secondSimBundle.drive.setPose(parkPose);
			secondSimBundle.drive.stop();
		} // End parkSecondSimOffField

		@Override
		public void logSecondSimParkedPose(boolean redAllianceForLogKey) {
			if (secondSimBundle == null) {
				return;
			}
			Pose2d loggedPose = secondSimBundle.driveSimulation.getSimulatedDriveTrainPose();
			Logger.recordOutput(SecondSimRobotOutputs.fieldSimulationRobotPositionKey(redAllianceForLogKey), loggedPose);
			field.getObject("robotPoseSim2").setPose(loggedPose);
		} // End logSecondSimParkedPose

		@Override
		public void cancelSecondSimShootWhenReadyAndStopDrive() {
			if (secondSimBundle == null) {
				return;
			}
			CommandScheduler.getInstance().cancel(secondSimBundle.shootWhenReady);
			secondSimBundle.drive.setDefaultCommand(Commands.run(secondSimBundle.drive::stop, secondSimBundle.drive));
		} // End cancelSecondSimShootWhenReadyAndStopDrive

		@Override
		public void restoreSecondSimTeleopDefaultCommand() {
			if (secondSimBundle == null) {
				return;
			}
			secondSimBundle.drive.setDefaultCommand(secondSimBundle.teleopDrive);
		} // End restoreSecondSimTeleopDefaultCommand
	} // End SimSecondRobotHostImpl
}
