package frc.robot.simulation;

/** RobotContainer hooks for {@link SimSecondRobotSession} (build, pose, park). */
public interface SimSecondRobotHost {

	public SecondSimRobotBundle getSecondSimBundle();

	public void setSecondSimBundle(SecondSimRobotBundle bundle);

	/** Create subsystems + Maple sim; not yet {@link #finishSecondSimSetup(SecondSimRobotBundle)}. */
	public SecondSimRobotBundle buildSecondSimBundle();

	public void finishSecondSimSetup(SecondSimRobotBundle bundle);

	public void applySecondSimPoseFromStem(String stem, boolean redAlliance);

	public void parkSecondSimOffField();

	/**
	 * @param redAllianceForLogKey alliance used for {@link SecondSimRobotOutputs#fieldSimulationRobotPositionKey(boolean)}
	 *        (pass the mode before disable; session clears {@code redAlliance} before this runs).
	 */
	public void logSecondSimParkedPose(boolean redAllianceForLogKey);

	public void cancelSecondSimShootWhenReadyAndStopDrive();

	public void restoreSecondSimTeleopDefaultCommand();
} // End SimSecondRobotHost
