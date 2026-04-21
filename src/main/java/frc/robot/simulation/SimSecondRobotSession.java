package frc.robot.simulation;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Dashboard mode + lifecycle for the optional second sim robot. Host performs IO/subsystem wiring; this class only
 * tracks mode and when to re-apply starting pose. Mode is read straight from the NT {@code selected} entry so that
 * programmatic local writes (e.g. the reset-to-defaults button) are honored immediately — {@link SendableChooser}
 * only updates its internal selection on remote NT value events and would otherwise lag our local writes.
 */
public final class SimSecondRobotSession {

	public static final String OPTION_DISABLE = "Disable";
	public static final String OPTION_BLUE = "Blue Alliance";
	public static final String OPTION_RED = "Red Alliance";

	/** Dashboard subtable name under SmartDashboard for the Second-Sim mode chooser. */
	public static final String DASHBOARD_KEY = "Second Sim Robot";

	private final SendableChooser<String> modeChooser = new SendableChooser<>();

	private Mode lastMode = Mode.DISABLE;

	private boolean driveEnabledFromDashboard;
	private boolean redAlliance;

	public SimSecondRobotSession() {
		modeChooser.setDefaultOption(OPTION_DISABLE, OPTION_DISABLE);
		modeChooser.addOption(OPTION_BLUE, OPTION_BLUE);
		modeChooser.addOption(OPTION_RED, OPTION_RED);
	} // End SimSecondRobotSession Constructor

	public SendableChooser<String> getModeChooser() {
		return modeChooser;
	} // End getModeChooser

	public boolean isDriveEnabledFromDashboard() {
		return driveEnabledFromDashboard;
	} // End isDriveEnabledFromDashboard

	public boolean isRedAlliance() {
		return redAlliance;
	} // End isRedAlliance

	/** Writes {@link #OPTION_DISABLE} to the mode chooser's {@code selected} NT entry. */
	public static void forceModeDisableViaNt() {
		NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(DASHBOARD_KEY)
				.getEntry("selected")
				.setString(OPTION_DISABLE);
	} // End forceModeDisableViaNt

	public void poll(SimSecondRobotHost host, Supplier<String> secondSimStemSupplier) {
		Mode mode = Mode.fromSelection(readNtSelectedMode());

		if (mode == Mode.DISABLE) {
			boolean transitioningToDisable = host.getSecondSimBundle() != null && lastMode != Mode.DISABLE;
			driveEnabledFromDashboard = false;
			redAlliance = false;
			if (transitioningToDisable) {
				host.cancelSecondSimShootWhenReadyAndStopDrive();
			}
			host.parkSecondSimOffField();
			if (transitioningToDisable) {
				host.logSecondSimParkedPose(lastMode == Mode.RED);
			}
			lastMode = mode;
			return;
		}
		else if (lastMode == Mode.BLUE && mode == Mode.RED) {
			host.parkSecondSimOffField();
			host.logSecondSimParkedPose(false);
		} else if (lastMode == Mode.RED && mode == Mode.BLUE) {
			host.parkSecondSimOffField();
			host.logSecondSimParkedPose(true);
		}


		driveEnabledFromDashboard = true;
		redAlliance = (mode == Mode.RED);

		if (host.getSecondSimBundle() == null) {
			host.setSecondSimBundle(host.buildSecondSimBundle());
			host.finishSecondSimSetup(host.getSecondSimBundle());
			host.applySecondSimPoseFromStem(secondSimStemSupplier.get(), redAlliance);
		} else if (lastMode == Mode.DISABLE) {
			host.restoreSecondSimTeleopDefaultCommand();
			host.applySecondSimPoseFromStem(secondSimStemSupplier.get(), redAlliance);
		} else if (lastMode != mode) {
			host.applySecondSimPoseFromStem(secondSimStemSupplier.get(), redAlliance);
		}

		lastMode = mode;
	} // End poll

	/** Reads the currently-selected mode string from NT directly (bypasses {@link SendableChooser#getSelected()}). */
	private static String readNtSelectedMode() {
		String ntValue = NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(DASHBOARD_KEY)
				.getEntry("selected")
				.getString("");
		return (ntValue == null || ntValue.isEmpty()) ? OPTION_DISABLE : ntValue;
	} // End readNtSelectedMode

	private enum Mode {
		DISABLE,
		BLUE,
		RED;

		static Mode fromSelection(String selected) {
			if (OPTION_BLUE.equals(selected)) {
				return BLUE;
			}
			if (OPTION_RED.equals(selected)) {
				return RED;
			}
			return DISABLE;
		}
	} // End Mode
} // End SimSecondRobotSession
