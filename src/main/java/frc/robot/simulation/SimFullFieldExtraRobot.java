package frc.robot.simulation;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import frc.robot.subsystems.drive.Drive;

/** Maple sim + drive for one full-field extra robot (PathPlanner start only). Carries the fixed chooser role it plays. */
public final class SimFullFieldExtraRobot {

	public SwerveDriveSimulation driveSimulation;
	public Drive drive;
	/** Fixed-role id in {@link SimStartingPoseFullFieldSim} (e.g. {@code ROLE_BLUE_2}, {@code ROLE_RED_3}). */
	public int role;
	/**
	 * {@link FuelSim} registered-robot index after the sim body is added; {@code -1} when fuel sim is off or not yet
	 * registered.
	 */
	public int fuelRobotIndex = -1;
} // End SimFullFieldExtraRobot
