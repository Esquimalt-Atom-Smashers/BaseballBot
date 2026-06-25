// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Subsystems.*;
import frc.robot.Subsystems.motor.TalonBrushedMotor;
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
	private boolean isCompetition = true;

	// Subsystems Toggle
	private boolean isDriveEnabled 		= true;
	private boolean isFlywheelEnabled = true;

	// Subsystems
	private Drive drive;
	@SuppressWarnings("unused")

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
    // Initialize Subsystems based on mode (REAL, SIM, or REPLAY)
		switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
			case REAL:
				if (isDriveEnabled) {
					TalonBrushedMotor leftMotor = new TalonBrushedMotor(1, false);
					drive = new Drive(leftMotor);
				} else {
					drive = new Drive(null);
				}

				// Subsystems
				break;

			// Sim robot, instantiate physics sim IO implementations
			case SIM:
				break;

			// Replayed Robot, disable IO implementations
			default:
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
	private void configureDriverBindings() {
		drive.setDefaultCommand(
			Commands.run(
				() -> {drive.arcadeDrive(
					driverController.getLeftY(),
					driverController.getRightX()
				);},
				drive
			)
		);
	} // End configureDriverBindings

  /** Configure Operator controls. */
  private void configureOperatorBindings(boolean enableOperatorControls) {
  } // End configureOperatorBindings

	/// -----------------------------------------------------------------------------------------------------------------
	/// --------------------------------------------- Other Useful Methods ----------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
	/** Idle all Subsystems.  */
	public void idleAllSubsystems() {
		CommandScheduler.getInstance().cancelAll();
	} // End idleBallHandling
}
