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

import frc.robot.subsystems.drive.*;

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
	private boolean isFlywheelEnabled = true;

	// Subsystems
	private final Drive drive;
	@SuppressWarnings("unused")
	private final Flywheel flywheel;

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

				// Subsystems
				break;

			// Sim robot, instantiate physics sim IO implementations
			case SIM:
				break;

			// Replayed Robot, disable IO implementations
			default
				break;
			}


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


		
		driverController.rightBumper().and(driverTriggerGate).onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.setIdleState(), intake),
				Commands.runOnce(() -> intake.setIntakingState(), intake),
				() -> intake.getState() == Intake.State.INTAKING));

	} // End configureDriverBindings

  /** Configure Operator controls. */
  private void configureOperatorBindings(boolean enableOperatorControls) {
  } // End configureOperatorBindings

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
}
