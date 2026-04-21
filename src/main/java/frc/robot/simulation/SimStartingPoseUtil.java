package frc.robot.simulation;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/** PathPlanner deploy starting paths: blue-frame poses for each stem. */
public final class SimStartingPoseUtil {

	private SimStartingPoseUtil() {} // End SimStartingPoseUtil Constructor

	public static final String STEM_CENTER = "StartingPosition-Center";
	public static final String STEM_CENTER_LEFT = "StartingPosition-CenterLeft";
	public static final String STEM_CENTER_RIGHT = "StartingPosition-CenterRight";
	public static final String STEM_LEFT = "StartingPosition-Left";
	public static final String STEM_RIGHT = "StartingPosition-Right";
	public static final String STEM_SWEEP_LEFT = "StartingPositionSweep-Left";
	public static final String STEM_SWEEP_RIGHT = "StartingPositionSweep-Right";

	public static final String[] PATH_STEMS = {
		STEM_CENTER,
		STEM_CENTER_LEFT,
		STEM_CENTER_RIGHT,
		STEM_LEFT,
		STEM_RIGHT,
		STEM_SWEEP_LEFT,
		STEM_SWEEP_RIGHT,
	};

	/** Holonomic start pose for the path; flipped for red alliance. */
	public static Pose2d poseForStem(String stem, boolean redAlliance) {
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile(stem);
			if (redAlliance) {
				path = path.flipPath();
			}
			return path.getStartingHolonomicPose()
					.orElseThrow(() -> new IllegalStateException("missing idealStartingState"));
		} catch (Exception e) {
			DriverStation.reportError("SimStartingPoseUtil poseForStem " + stem + ": " + e.getMessage(), false);
			return new Pose2d();
		}
	} // End poseForStem
}
