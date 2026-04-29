package frc.robot.simulation;

/**
 * Unified gamepad input for full-field extra sim {@linkplain SimFullFieldExtraBehaviourSim human control}: same
 * stick/trigger semantics as {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController}, with an alternate path
 * using {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick} / {@link edu.wpi.first.wpilibj.GenericHID} raw axes
 * for roles that must use DirectInput because Windows only delivers XInput to four pads at once.
 */
public interface HumanControlGamepad {

	double getLeftX();

	double getLeftY();

	double getRightX();

	double getRightTriggerAxis();

	boolean getAButton();

	/** Blue 3 and Red 3 use DirectInput-style mapping on the Driver Station PC. */
	static boolean usesDirectInputForFullFieldRole(int role) {
		return role == SimStartingPoseFullFieldSim.ROLE_BLUE_3 || role == SimStartingPoseFullFieldSim.ROLE_RED_3;
	} // End usesDirectInputForFullFieldRole

	static HumanControlGamepad forFullFieldExtraRole(int port, int role) {
		return forPort(port, usesDirectInputForFullFieldRole(role));
	} // End forFullFieldExtraRole

	static HumanControlGamepad forPort(int port, boolean useDirectInput) {
		return HumanControlGamepadImpl.forPort(port, useDirectInput);
	} // End forPort
} // End HumanControlGamepad
