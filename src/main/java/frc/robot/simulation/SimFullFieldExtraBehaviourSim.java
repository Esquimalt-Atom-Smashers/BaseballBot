package frc.robot.simulation;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.util.LocalADStarAK;

/**
 * SIM-only behavior chooser manager for full-field extra robots.
 *
 * <p>This class intentionally excludes Primary and Second-Sim because those are human-controlled robots.
 */
public final class SimFullFieldExtraBehaviourSim {

	public static final String OPTION_DO_NOTHING = "Do Nothing";
	public static final String OPTION_DEFENSE_BLOCK = "Defense (Block)";
	/** Proportional gain used to hold X on the trench-neutral block line. */
	private static final double kDefenseBlockPathXP = 5.0;
	/** Proportional gain used to match defender Y to the tracked robot Y. */
	private static final double kDefenseBlockPathYP = 4.2;
	/** Linear speed clamp for defense-block steering. */
	private static final double kDefenseBlockPathMaxLinearMetersPerSec = 2.5;
	/** Extra stand-off from trench neutral edge so defenders stay just outside the trench lane. */
	private static final double kDefenseBlockNeutralSideOffsetMeters = 0.6;
	/** If target moves this much, force immediate replan. */
	private static final double kDefenseBlockTargetReplanDistanceMeters = 0.25;
	/** If robot strays this far from cached path start, force immediate replan. */
	private static final double kDefenseBlockStartReplanDistanceMeters = 0.35;
	/** Waypoint tolerance to advance to next point. */
	private static final double kDefenseBlockWaypointToleranceMeters = 0.35;
	/** Prime replan periods per role (different primes prevent collisions). */
	private static final int[] kDefenseBlockPrimeReplanTicks = {23, 19, 17, 13, 11};
	/** Role-specific phase offset so equal primes never align (extra safety). */
	private static final int[] kDefenseBlockPrimePhaseTicks = {0, 3, 5, 7, 11};
	/** Path constraints used by defense block pathfinder. */
	private static final PathConstraints kDefenseBlockPathConstraints =
			new PathConstraints(5.0, 3.0, 2.5 * Math.PI, 3.0 * Math.PI);

	private static final int[] EXTRA_ROLES = {
			SimStartingPoseFullFieldSim.ROLE_BLUE_2,
			SimStartingPoseFullFieldSim.ROLE_BLUE_3,
			SimStartingPoseFullFieldSim.ROLE_RED_1,
			SimStartingPoseFullFieldSim.ROLE_RED_2,
			SimStartingPoseFullFieldSim.ROLE_RED_3
	};

	private static final String[] EXTRA_ROLE_NAMES = {
			"Blue-2",
			"Blue-3",
			"Red-1",
			"Red-2",
			"Red-3"
	};

	private static final String[] EXTRA_ROLE_DEFAULT_BEHAVIOR = {
			OPTION_DO_NOTHING,    // Blue-2 (Cycle comes in later phase)
			OPTION_DEFENSE_BLOCK, // Blue-3
			OPTION_DEFENSE_BLOCK, // Red-1
			OPTION_DEFENSE_BLOCK, // Red-2 (Aggressive comes in later phase)
			OPTION_DO_NOTHING     // Red-3 (Cycle comes in later phase)
	};

	private static final int[][] EXTRAS_ROLES_BY_LAYOUT = {
			{
					SimStartingPoseFullFieldSim.ROLE_BLUE_2,
					SimStartingPoseFullFieldSim.ROLE_BLUE_3,
					SimStartingPoseFullFieldSim.ROLE_RED_1,
					SimStartingPoseFullFieldSim.ROLE_RED_2,
					SimStartingPoseFullFieldSim.ROLE_RED_3
			}, // Layout 0: no Second-Sim
			{
					SimStartingPoseFullFieldSim.ROLE_BLUE_3,
					SimStartingPoseFullFieldSim.ROLE_RED_1,
					SimStartingPoseFullFieldSim.ROLE_RED_2,
					SimStartingPoseFullFieldSim.ROLE_RED_3
			}, // Layout 1: Second-Sim on Blue
			{
					SimStartingPoseFullFieldSim.ROLE_BLUE_2,
					SimStartingPoseFullFieldSim.ROLE_BLUE_3,
					SimStartingPoseFullFieldSim.ROLE_RED_2,
					SimStartingPoseFullFieldSim.ROLE_RED_3
			}, // Layout 2: Second-Sim on Red
	};

	private final Map<Integer, LocalADStarAK> defensePathfinderByRole = new HashMap<>();
	private final Map<Integer, PathPlannerPath> defensePathByRole = new HashMap<>();
	private final Map<Integer, Pose2d> defenseLastGoalByRole = new HashMap<>();
	private int behaviorTickCounter = 0;

	public void init() {
		for (int index = 0; index < EXTRA_ROLES.length; index++) {
			SendableChooser<String> behaviorChooser = new SendableChooser<>();
			behaviorChooser.setDefaultOption(EXTRA_ROLE_DEFAULT_BEHAVIOR[index], EXTRA_ROLE_DEFAULT_BEHAVIOR[index]);
			behaviorChooser.addOption(OPTION_DO_NOTHING, OPTION_DO_NOTHING);
			behaviorChooser.addOption(OPTION_DEFENSE_BLOCK, OPTION_DEFENSE_BLOCK);
			SmartDashboard.putData(dashboardKeyForExtraIndex(index), behaviorChooser);
		}
	} // End init

	/** Returns selected behavior for the fixed extra role. */
	public String selectedBehaviorForRole(int role) {
		int index = indexForRole(role);
		if (index < 0) {
			return OPTION_DO_NOTHING;
		}
		String ntValue = NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(dashboardKeyForExtraIndex(index))
				.getEntry("selected")
				.getString("");
		if (ntValue == null || ntValue.isEmpty()) {
			return EXTRA_ROLE_DEFAULT_BEHAVIOR[index];
		}
		return ntValue;
	} // End selectedBehaviorForRole

	/** Resets extra behavior chooser selections when the shared sim reset button is pressed. */
	public void pollResetToDefaults(boolean simMode) {
		if (!simMode) {
			return;
		}
		if (!SmartDashboard.getBoolean("SimStartingPose/ResetToDefaults", false)) {
			return;
		}
		for (int index = 0; index < EXTRA_ROLES.length; index++) {
			forceBehaviorSelectionByIndex(index, EXTRA_ROLE_DEFAULT_BEHAVIOR[index]);
		}
	} // End pollResetToDefaults

	/** Iterates active extra robots for current layout and passes (role, robot) to consumer. */
	public void forEachActiveExtra(
			SimFullFieldExtraRobot[] extraRobotsByPool,
			boolean extrasEnabled,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance,
			BiConsumer<Integer, SimFullFieldExtraRobot> consumer) {
		if (!extrasEnabled || extraRobotsByPool == null || consumer == null) {
			return;
		}
		int layout = layoutKey(secondSimEnabled, secondSimRedAlliance);
		int[] rolesForLayout = EXTRAS_ROLES_BY_LAYOUT[layout];
		for (int poolIdx = 0; poolIdx < rolesForLayout.length; poolIdx++) {
			if (poolIdx >= extraRobotsByPool.length) {
				break;
			}
			SimFullFieldExtraRobot extraRobot = extraRobotsByPool[poolIdx];
			if (extraRobot == null) {
				continue;
			}
			consumer.accept(rolesForLayout[poolIdx], extraRobot);
		}
	} // End forEachActiveExtra

	/**
	 * Updates extra robot behaviors.
	 *
	 * <p>Red defenders track Primary. Blue defenders track Second-Sim when it is on red, otherwise Red-2.
	 */
	public void updateExtraRobotBehaviors(
			SimFullFieldExtraRobot[] extraRobotsByPool,
			boolean extrasEnabled,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance,
			boolean teleopEnabled,
			Pose2d primaryPose,
			Pose2d secondSimPose) {
		Map<Integer, SimFullFieldExtraRobot> activeExtraByRole = new HashMap<>();
		forEachActiveExtra(
				extraRobotsByPool,
				extrasEnabled,
				secondSimEnabled,
				secondSimRedAlliance,
				(role, extraRobot) -> activeExtraByRole.put(role, extraRobot));

		if (!teleopEnabled) {
			for (SimFullFieldExtraRobot extraRobot : activeExtraByRole.values()) {
				extraRobot.drive.stop();
			}
			return;
		}
		behaviorTickCounter++;

		SimFullFieldExtraRobot red2ExtraRobot = activeExtraByRole.get(SimStartingPoseFullFieldSim.ROLE_RED_2);
		Pose2d red2Pose = red2ExtraRobot != null ? red2ExtraRobot.driveSimulation.getSimulatedDriveTrainPose() : null;

		for (int role : EXTRA_ROLES) {
			SimFullFieldExtraRobot extraRobot = activeExtraByRole.get(role);
			if (extraRobot == null) {
				continue;
			}

			String selectedBehavior = selectedBehaviorForRole(role);
			Logger.recordOutput("SimFullFieldExtra/" + role + "/SelectedBehavior", selectedBehavior);

			if (OPTION_DEFENSE_BLOCK.equals(selectedBehavior)) {
				Pose2d targetPose = defenseBlockTargetPose(role, primaryPose, secondSimPose, red2Pose, secondSimEnabled, secondSimRedAlliance);
				if (targetPose == null) {
					extraRobot.drive.stop();
					continue;
				}
				runDefenseBlockPathfind(role, extraRobot, roleIsRedAlliance(role), targetPose);
			} else {
				// Includes explicit Do Nothing plus any future, unimplemented options.
				extraRobot.drive.stop();
			}
		}
	} // End updateExtraRobotBehaviors

	/** 0 = no Second-Sim, 1 = Second-Sim on Blue, 2 = Second-Sim on Red. */
	private static int layoutKey(boolean secondSimEnabled, boolean secondSimRedAlliance) {
		if (!secondSimEnabled) {
			return 0;
		}
		return secondSimRedAlliance ? 2 : 1;
	} // End layoutKey

	/** Returns the tracked target pose for defense block. */
	private static Pose2d defenseBlockTargetPose(
			int role,
			Pose2d primaryPose,
			Pose2d secondSimPose,
			Pose2d red2Pose,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance) {
		if (roleIsRedAlliance(role)) {
			return primaryPose;
		}
		if (secondSimEnabled && secondSimRedAlliance && secondSimPose != null) {
			return secondSimPose;
		}
		return red2Pose;
	} // End defenseBlockTargetPose

	/** Runs defense block using navgrid pathfinding with prime-tick replan staggering. */
	private void runDefenseBlockPathfind(
			int role,
			SimFullFieldExtraRobot extraRobot,
			boolean defenderIsRedAlliance,
			Pose2d trackedRobotPose) {
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		Pose2d goalPose = new Pose2d(
				trenchNeutralSideXForOpposingAlliance(defenderIsRedAlliance),
				trackedRobotPose.getY(),
				selfPose.getRotation());

		LocalADStarAK pathfinder = defensePathfinderByRole.computeIfAbsent(role, k -> new LocalADStarAK());
		PathPlannerPath cachedPath = defensePathByRole.get(role);
		Pose2d lastGoal = defenseLastGoalByRole.get(role);
		boolean scheduleReplan = shouldReplanForRole(role);
		boolean targetMoved = lastGoal == null
				|| lastGoal.getTranslation().getDistance(goalPose.getTranslation()) > kDefenseBlockTargetReplanDistanceMeters;
		List<PathPoint> cachedPoints = cachedPath != null ? cachedPath.getAllPathPoints() : null;
		boolean robotMovedFromStart = cachedPath == null
				|| cachedPoints == null
				|| cachedPoints.isEmpty()
				|| cachedPoints.get(0).position.getDistance(selfPose.getTranslation()) > kDefenseBlockStartReplanDistanceMeters;
		if (scheduleReplan || targetMoved || robotMovedFromStart) {
			pathfinder.setStartPosition(selfPose.getTranslation());
			pathfinder.setGoalPosition(goalPose.getTranslation());
			PathPlannerPath newPath = pathfinder.getCurrentPath(
					kDefenseBlockPathConstraints,
					new GoalEndState(0.0, Rotation2d.fromRadians(0.0)));
			if (newPath != null) {
				defensePathByRole.put(role, newPath);
				defenseLastGoalByRole.put(role, goalPose);
				cachedPath = newPath;
			}
		}

		Pose2d driveTargetPose = goalPose;
		if (cachedPath != null) {
			driveTargetPose = choosePathTargetPose(cachedPath, selfPose, goalPose);
		}
		driveTowardPose(extraRobot, selfPose, driveTargetPose);
	} // End runDefenseBlockPathfind

	/** Role-specific prime-tick schedule to spread replans across simulation ticks. */
	private boolean shouldReplanForRole(int role) {
		int idx = indexForRole(role);
		if (idx < 0 || idx >= kDefenseBlockPrimeReplanTicks.length) {
			return true;
		}
		int period = kDefenseBlockPrimeReplanTicks[idx];
		int phase = kDefenseBlockPrimePhaseTicks[idx] % period;
		return behaviorTickCounter % period == phase;
	} // End shouldReplanForRole

	/** Selects a nearby lookahead waypoint on the current path. */
	private static Pose2d choosePathTargetPose(PathPlannerPath path, Pose2d selfPose, Pose2d fallbackGoalPose) {
		List<PathPoint> points = path.getAllPathPoints();
		if (points.isEmpty()) {
			return fallbackGoalPose;
		}
		int bestIndex = 0;
		double bestDistance = Double.POSITIVE_INFINITY;
		for (int i = 0; i < points.size(); i++) {
			double distance = points.get(i).position.getDistance(selfPose.getTranslation());
			if (distance < bestDistance) {
				bestDistance = distance;
				bestIndex = i;
			}
		}
		int targetIndex = Math.min(points.size() - 1, bestIndex + 2);
		if (bestDistance < kDefenseBlockWaypointToleranceMeters) {
			targetIndex = Math.min(points.size() - 1, targetIndex + 1);
		}
		return new Pose2d(points.get(targetIndex).position, fallbackGoalPose.getRotation());
	} // End choosePathTargetPose

	/** Drives field-centrically toward a pose target. */
	private static void driveTowardPose(SimFullFieldExtraRobot extraRobot, Pose2d selfPose, Pose2d targetPose) {
		double xErrorMeters = targetPose.getX() - selfPose.getX();
		double yErrorMeters = targetPose.getY() - selfPose.getY();
		double vxMetersPerSec = MathUtil.clamp(
				xErrorMeters * kDefenseBlockPathXP,
				-kDefenseBlockPathMaxLinearMetersPerSec,
				kDefenseBlockPathMaxLinearMetersPerSec);
		double vyMetersPerSec = MathUtil.clamp(
				yErrorMeters * kDefenseBlockPathYP,
				-kDefenseBlockPathMaxLinearMetersPerSec,
				kDefenseBlockPathMaxLinearMetersPerSec);
		extraRobot.drive.driveFieldCentric(vxMetersPerSec, vyMetersPerSec, 0.0);
	} // End driveTowardPose

	/** Returns the neutral-zone-side X of the opposing alliance trench, including a small offset. */
	private static double trenchNeutralSideXForOpposingAlliance(boolean defenderIsRedAlliance) {
		double blueNeutralSideX = FieldConstants.TRENCH_BUMP_X_M
				+ (FieldConstants.TRENCH_BUMP_LENGTH_M / 2.0)
				+ kDefenseBlockNeutralSideOffsetMeters;
		if (defenderIsRedAlliance) {
			return blueNeutralSideX;
		}
		return FieldConstants.FIELD_LENGTH_M - blueNeutralSideX;
	} // End trenchNeutralSideXForOpposingAlliance

	/** True when the fixed extra role belongs to red alliance. */
	private static boolean roleIsRedAlliance(int role) {
		return role == SimStartingPoseFullFieldSim.ROLE_RED_1
				|| role == SimStartingPoseFullFieldSim.ROLE_RED_2
				|| role == SimStartingPoseFullFieldSim.ROLE_RED_3;
	} // End roleIsRedAlliance

	private static int indexForRole(int role) {
		for (int i = 0; i < EXTRA_ROLES.length; i++) {
			if (EXTRA_ROLES[i] == role) {
				return i;
			}
		}
		return -1;
	} // End indexForRole

	private static String dashboardKeyForExtraIndex(int index) {
		return "SimBehavior/" + EXTRA_ROLE_NAMES[index];
	} // End dashboardKeyForExtraIndex

	private static void forceBehaviorSelectionByIndex(int index, String behavior) {
		NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(dashboardKeyForExtraIndex(index))
				.getEntry("selected")
				.setString(behavior);
	} // End forceBehaviorSelectionByIndex
} // End SimFullFieldExtraBehaviorSim
