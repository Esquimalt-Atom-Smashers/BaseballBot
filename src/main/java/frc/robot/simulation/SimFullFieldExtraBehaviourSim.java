package frc.robot.simulation;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LocalADStarAK;

/**
 * SIM-only behavior chooser manager for full-field extra robots.
 *
 * <p>This class intentionally excludes Primary and Second-Sim because those are human-controlled robots.
 *
 * <p>Cycle behaviors {@value #OPTION_CYCLE_BUMP} and {@value #OPTION_CYCLE_TRENCH} share the same two-path PathPlanner
 * sim loop ({@code pathOne} then {@code pathTwo}) and hub scoring rules.
 */
public final class SimFullFieldExtraBehaviourSim {

	public static final String OPTION_DO_NOTHING = "Do Nothing";
	public static final String OPTION_DEFENSE_BLOCK = "Defense (Block)";
	public static final String OPTION_DEFENSE_AGGRESSIVE = "Defense (Aggressive)";
	/** Bump loop: pathfind to authored path starts, follow spline samples, hub score when shift and fuel allow. */
	public static final String OPTION_CYCLE_BUMP = "Cycle (Bump)";
	/** Trench loop: {@code AllianceWallSweep} then {@code NeutralZoneSweep}; same two-path hub scoring as bump. */
	public static final String OPTION_CYCLE_TRENCH = "Cycle (Trench)";

	// Staggered replanning times to avoid collisions
	/** Prime replan periods per role (different primes prevent collisions). */
	private static final int[] kPrimeReplanTicks = {23, 19, 17, 13, 11};
	/** Role-specific phase offsets for the shared per-robot replan schedule. */
	private static final int[] kPrimePhaseTicks = {0, 3, 5, 7, 11};

	// Defense Block Config
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
	/** Path constraints used by defense block pathfinder. */
	private static final PathConstraints kDefenseBlockPathConstraints =
			new PathConstraints(5.0, 3.0, 2.5 * Math.PI, 3.0 * Math.PI);

	// Defense Aggressive Config
	/** Proportional gain used for aggressive chase/backoff X control. */
	private static final double kDefenseAggressivePathXP = 10.0;
	/** Proportional gain used for aggressive chase/backoff Y control. */
	private static final double kDefenseAggressivePathYP = 10.0;
	/** Linear speed clamp for aggressive chase/backoff steering. */
	private static final double kDefenseAggressivePathMaxLinearMetersPerSec = 3.5;
	/** Path constraints used by aggressive defense pathfinder. */
	private static final PathConstraints kDefenseAggressivePathConstraints =
			new PathConstraints(5.5, 5.0, 3.0 * Math.PI, 3.5 * Math.PI);
	/** Distance threshold to count a ram contact attempt as completed. */
	private static final double kDefenseAggressiveRamSwitchDistanceMeters = 1.0;
	/** Backoff travel distance after a ram attempt. */
	private static final double kDefenseAggressiveBackoffDistanceMeters = 2.0;
	/** Delay after ram contact before switching to backoff (1.0s @ 20ms loop). */
	private static final int kDefenseAggressiveBackoffDelayTicks = 53;
	/** If backoff cannot complete within this time, abort and charge again (5.0s @ 20ms loop). */
	private static final int kDefenseAggressiveBackoffTimeoutTicks = 250;
	/** If separation exceeds this after contact, cancel countdown and resume chase. */
	private static final double kDefenseAggressiveContactReleaseDistanceMeters = 2.2;
	/** Position tolerance to finish backoff and start next ram. */
	private static final double kDefenseAggressiveBackoffArriveMeters = 0.2;
	/** Fallback backoff heading when target and robot overlap. */
	private static final double kDefenseAggressiveFallbackBackoffXSign = -1.0;

	// Two-path cycle: authored PathPlanner legs plus navgrid to each holonomic start.
	/** AD star constraints for reaching holonomic path starts. */
	private static final PathConstraints kTwoPathCyclePathConstraints =
			new PathConstraints(5.0, 5.0, 2.5 * Math.PI, 3.0 * Math.PI);
	/** Translation tolerance (m) to treat holonomic path start as reached. */
	private static final double kTwoPathCyclePathStartArriveMeters = 0.42;
	/** Advance follow index when this close to the current sample (m). */
	private static final double kTwoPathCycleFollowWaypointToleranceMeters = 0.42;
	/** XY error to field velocity gain for follow and path-start approach. */
	private static final double kTwoPathCycleFollowLinearP = 4.5;
	/** Heading error to omega gain for follow and path-start approach. */
	private static final double kTwoPathCycleFollowOmegaP = 5;
	/** Linear speed clamp during spline follow (m/s). */
	private static final double kTwoPathCycleFollowMaxLinearMetersPerSec = 3.0;
	/** Omega clamp during spline follow (rad/s). */
	private static final double kTwoPathCycleFollowMaxOmegaRadPerSec = 5;
	/**
	 * Minimum sim ticks between hub launch attempts during two-path cycle behaviors for full-field extras only.
	 * With a 20 ms robot loop, average launch rate is about 50 divided by this value (per second).
	 * Primary and second sim use {@link frc.robot.subsystems.shooter.ShooterSim} for shoot cadence instead.
	 */
	private static final int kTwoPathCycleScoreLaunchIntervalTicks = 5;
	/** Max |heading error| (rad) before a launch is allowed. */
	private static final double kTwoPathCycleScoreFacingToleranceRad = 0.14;
	/**
	 * Sim ticks without reaching holonomic path start before re-picking which pathfind leg to run from the nearer
	 * holonomic start (2.0 s when {@link #updateExtraRobotBehaviors} runs once per 20 ms robot period).
	 */
	private static final int kTwoPathCyclePathfindStartTimeoutTicks = 100;

	/** PathPlanner deploy file stem for bump cycle path one (blue alliance frame). */
	private static final String BUMP_CYCLE_PATH_ONE_DEPLOY_STEM = "Bump-AllianceToNeutral-Left";
	/** PathPlanner deploy file stem for bump cycle path two (blue alliance frame). */
	private static final String BUMP_CYCLE_PATH_TWO_DEPLOY_STEM = "Bump-NeutralToAlliance-Right";

	/** Loaded blue-frame first bump leg; null if load failed. */
	private static PathPlannerPath bumpPathOneBlueAuthoring;
	/** Loaded blue-frame second bump leg; null if load failed. */
	private static PathPlannerPath bumpPathTwoBlueAuthoring;
	/** Red alliance copy of {@link #bumpPathOneBlueAuthoring}; null if load failed. */
	private static PathPlannerPath bumpPathOneRedAuthoring;
	/** Red alliance copy of {@link #bumpPathTwoBlueAuthoring}; null if load failed. */
	private static PathPlannerPath bumpPathTwoRedAuthoring;
	/** True after first attempt to read bump paths from deploy. */
	private static boolean bumpPathsLoadAttempted;
	/** Non-null when bump path load threw; used for sim log only. */
	private static String bumpPathsLoadError;

	/** PathPlanner deploy file stem for trench cycle path one (blue alliance frame). */
	private static final String TRENCH_CYCLE_PATH_ONE_DEPLOY_STEM = "AllianceWallSweep";
	/** PathPlanner deploy file stem for trench cycle path two (blue alliance frame). */
	private static final String TRENCH_CYCLE_PATH_TWO_DEPLOY_STEM = "NeutralZoneSweep";

	/** Loaded blue-frame first trench leg; null if load failed. */
	private static PathPlannerPath trenchPathOneBlueAuthoring;
	/** Loaded blue-frame second trench leg; null if load failed. */
	private static PathPlannerPath trenchPathTwoBlueAuthoring;
	/** Red alliance copy of {@link #trenchPathOneBlueAuthoring}; null if load failed. */
	private static PathPlannerPath trenchPathOneRedAuthoring;
	/** Red alliance copy of {@link #trenchPathTwoBlueAuthoring}; null if load failed. */
	private static PathPlannerPath trenchPathTwoRedAuthoring;
	/** True after first attempt to read trench paths from deploy. */
	private static boolean trenchPathsLoadAttempted;
	/** Non-null when trench path load threw; used for sim log only. */
	private static String trenchPathsLoadError;

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
			OPTION_DO_NOTHING,    // Blue-2
			OPTION_DEFENSE_BLOCK, // Blue-3
			OPTION_DEFENSE_BLOCK, // Red-1
			OPTION_DEFENSE_AGGRESSIVE, // Red-2
			OPTION_DO_NOTHING     // Red-3
	};

	private static final int[][] EXTRAS_ROLES_BY_LAYOUT = {
			EXTRA_ROLES, // Layout 0: no Second-Sim
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
	private final Map<Integer, LocalADStarAK> aggressivePathfinderByRole = new HashMap<>();
	private final Map<Integer, PathPlannerPath> aggressivePathByRole = new HashMap<>();
	private final Map<Integer, Pose2d> aggressiveLastGoalByRole = new HashMap<>();
	private final Map<Integer, Boolean> aggressiveBackingOffByRole = new HashMap<>();
	private final Map<Integer, Pose2d> aggressiveBackoffGoalByRole = new HashMap<>();
	private final Map<Integer, Integer> aggressiveBackoffStartTickByRole = new HashMap<>();
	private final Map<Integer, Integer> aggressiveRamContactTickByRole = new HashMap<>();
	private final Map<Integer, String> lastBehaviorByRole = new HashMap<>();
	private int behaviorTickCounter = 0;

	/** Phases for {@link #runTwoPathCycleSim}: pathfind and follow each leg, then optional hub scoring. */
	private enum ExtraSimTwoPathCyclePhase {
		/** Navgrid to the first leg holonomic start. */
		PATHFIND_TO_PATHONE,
		/** Field-centric tracking of the first leg samples. */
		FOLLOW_PATHONE,
		/** Navgrid to the second leg holonomic start. */
		PATHFIND_TO_PATHTWO,
		/** Field-centric tracking of the second leg samples. */
		FOLLOW_PATHTWO,
		/** Turn toward hub and launch carried fuel when shift timing allows. */
		SCORE_HUB
	}

	private final Map<Integer, ExtraSimTwoPathCyclePhase> twoPathCyclePhaseByRole = new HashMap<>();
	private final Map<Integer, List<Translation2d>> twoPathCycleFollowPointsByRole = new HashMap<>();
	private final Map<Integer, Integer> twoPathCycleFollowIndexByRole = new HashMap<>();
	private final Map<Integer, LocalADStarAK> twoPathCyclePathfinderByRole = new HashMap<>();
	private final Map<Integer, PathPlannerPath> twoPathCyclePathByRole = new HashMap<>();
	private final Map<Integer, Pose2d> twoPathCycleLastGoalByRole = new HashMap<>();
	private final Map<Integer, Integer> twoPathCycleLastLaunchTickByRole = new HashMap<>();
	/** {@link #behaviorTickCounter} when the current pathfind leg began (cleared on arrive, timeout, behavior change). */
	private final Map<Integer, Integer> twoPathCyclePathfindLegStartTickByRole = new HashMap<>();

	public void init() {
		for (int index = 0; index < EXTRA_ROLES.length; index++) {
			SendableChooser<String> behaviorChooser = new SendableChooser<>();
			behaviorChooser.setDefaultOption(EXTRA_ROLE_DEFAULT_BEHAVIOR[index], EXTRA_ROLE_DEFAULT_BEHAVIOR[index]);
			behaviorChooser.addOption(OPTION_DO_NOTHING, OPTION_DO_NOTHING);
			behaviorChooser.addOption(OPTION_DEFENSE_BLOCK, OPTION_DEFENSE_BLOCK);
			behaviorChooser.addOption(OPTION_DEFENSE_AGGRESSIVE, OPTION_DEFENSE_AGGRESSIVE);
			behaviorChooser.addOption(OPTION_CYCLE_BUMP, OPTION_CYCLE_BUMP);
			behaviorChooser.addOption(OPTION_CYCLE_TRENCH, OPTION_CYCLE_TRENCH);
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
		if (ntValue.isEmpty()) {
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
	 *
	 * @param fuelSim registered fuel simulation instance, or null when fuel sim is off
	 * @param fuelSimEnabled when false, two-path cycle hub scoring treats carried fuel count as zero
	 *
	 * <p>When {@code teleopEnabled} is false, active extras stop and all extra motion caches (including two-path cycle
	 * phase) are cleared so re-enable cold-starts pathfind instead of resuming a follow leg.
	 */
	public void updateExtraRobotBehaviors(
			SimFullFieldExtraRobot[] extraRobotsByPool,
			boolean extrasEnabled,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance,
			boolean teleopEnabled,
			Pose2d primaryPose,
			Pose2d secondSimPose,
			FuelSim fuelSim,
			boolean fuelSimEnabled) {
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
			for (int role : EXTRA_ROLES) {
				clearTransientExtraMotionCaches(role);
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
			resetRoleStateOnBehaviorChange(role, selectedBehavior);

			if (OPTION_DEFENSE_BLOCK.equals(selectedBehavior)) {
				Pose2d targetPose = defenseBlockTargetPose(role, primaryPose, secondSimPose, red2Pose, secondSimEnabled, secondSimRedAlliance);
				if (targetPose == null) {
					extraRobot.drive.stop();
					continue;
				}
				runDefenseBlockPathfind(role, extraRobot, roleIsRedAlliance(role), targetPose);
			} else if (OPTION_DEFENSE_AGGRESSIVE.equals(selectedBehavior)) {
				Pose2d targetPose = defenseBlockTargetPose(role, primaryPose, secondSimPose, red2Pose, secondSimEnabled, secondSimRedAlliance);
				if (targetPose == null) {
					extraRobot.drive.stop();
					continue;
				}
				runDefenseAggressivePathfind(role, extraRobot, roleIsRedAlliance(role), targetPose);
			} else if (OPTION_CYCLE_BUMP.equals(selectedBehavior)) {
				runBumpCycleSim(role, extraRobot, fuelSim, fuelSimEnabled);
			} else if (OPTION_CYCLE_TRENCH.equals(selectedBehavior)) {
				runTrenchCycleSim(role, extraRobot, fuelSim, fuelSimEnabled);
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
		PathPlannerPath cachedPath = replanAndGetPath(
				role,
				selfPose,
				goalPose,
				kDefenseBlockPathConstraints,
				kPrimeReplanTicks,
				kPrimePhaseTicks,
				defensePathfinderByRole,
				defensePathByRole,
				defenseLastGoalByRole);
		Pose2d driveTargetPose = goalPose;
		if (cachedPath != null) {
			driveTargetPose = choosePathTargetPose(cachedPath, selfPose, goalPose);
		}
		driveTowardPoseProportional(
				extraRobot,
				selfPose,
				driveTargetPose,
				kDefenseBlockPathXP,
				kDefenseBlockPathYP,
				kDefenseBlockPathMaxLinearMetersPerSec);
	} // End runDefenseBlockPathfind

	/** Runs aggressive defense by looping between ram and 3m backoff. */
	private void runDefenseAggressivePathfind(
			int role,
			SimFullFieldExtraRobot extraRobot,
			boolean defenderIsRedAlliance,
			Pose2d trackedRobotPose) {
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		boolean backingOff = aggressiveBackingOffByRole.getOrDefault(role, false);
		Pose2d goalPose;

		if (!backingOff) {
			goalPose = trackedRobotPose;
			double distanceToTarget = selfPose.getTranslation().getDistance(trackedRobotPose.getTranslation());
			int contactTick = aggressiveRamContactTickByRole.getOrDefault(role, -1);

			// Latch contact once, then run a pure timer-based delay before backoff.
			if (contactTick < 0 && distanceToTarget <= kDefenseAggressiveRamSwitchDistanceMeters) {
				contactTick = behaviorTickCounter;
				aggressiveRamContactTickByRole.put(role, contactTick);
			}

			// If target separation grows too much, cancel latch and re-acquire contact.
			if (contactTick >= 0 && distanceToTarget > kDefenseAggressiveContactReleaseDistanceMeters) {
				aggressiveRamContactTickByRole.remove(role);
				contactTick = -1;
			}

			// Countdown continues even if distance wiggles around while pushing.
			if (contactTick >= 0 && behaviorTickCounter - contactTick >= kDefenseAggressiveBackoffDelayTicks) {
				Pose2d backoffGoal = computeAggressiveBackoffGoal(selfPose, trackedRobotPose, defenderIsRedAlliance);
				aggressiveBackoffGoalByRole.put(role, backoffGoal);
				aggressiveBackingOffByRole.put(role, true);
				aggressiveBackoffStartTickByRole.put(role, behaviorTickCounter);
				aggressiveRamContactTickByRole.remove(role);
				backingOff = true;
				goalPose = backoffGoal;
			}
		} else {
			goalPose = aggressiveBackoffGoalByRole.getOrDefault(
					role,
					computeAggressiveBackoffGoal(selfPose, trackedRobotPose, defenderIsRedAlliance));
			double distanceToBackoff = selfPose.getTranslation().getDistance(goalPose.getTranslation());
			int backoffStartTick = aggressiveBackoffStartTickByRole.getOrDefault(role, behaviorTickCounter);
			boolean backoffTimedOut = behaviorTickCounter - backoffStartTick >= kDefenseAggressiveBackoffTimeoutTicks;
			boolean backoffArrived = distanceToBackoff <= kDefenseAggressiveBackoffArriveMeters;
			// Arrive = finish backoff cleanly; timeout = abort stale backoff. Both resume chase.
			if (backoffArrived || backoffTimedOut) {
				aggressiveBackingOffByRole.put(role, false);
				aggressiveBackoffGoalByRole.remove(role);
				aggressiveBackoffStartTickByRole.remove(role);
				aggressiveRamContactTickByRole.remove(role);
				goalPose = trackedRobotPose;
			}
		}

		PathPlannerPath path = replanAndGetPath(
				role,
				selfPose,
				goalPose,
				kDefenseAggressivePathConstraints,
				kPrimeReplanTicks,
				kPrimePhaseTicks,
				aggressivePathfinderByRole,
				aggressivePathByRole,
				aggressiveLastGoalByRole);

		Pose2d driveTargetPose = goalPose;
		if (path != null) {
			driveTargetPose = choosePathTargetPose(path, selfPose, goalPose);
		}
		driveTowardPoseProportional(
				extraRobot,
				selfPose,
				driveTargetPose,
				kDefenseAggressivePathXP,
				kDefenseAggressivePathYP,
				kDefenseAggressivePathMaxLinearMetersPerSec);
	} // End runDefenseAggressivePathfind

	/** Role-specific prime-tick schedule to spread replans across simulation ticks. */
	private boolean shouldReplanForRole(int role, int[] replanTicks, int[] phaseTicks) {
		int idx = indexForRole(role);
		if (idx < 0 || idx >= replanTicks.length || idx >= phaseTicks.length) {
			return true;
		}
		int period = replanTicks[idx];
		int phase = phaseTicks[idx] % period;
		return behaviorTickCounter % period == phase;
	} // End shouldReplanForRole

	/** Replans path when schedule/goal/start thresholds require it, then returns cached/new path. */
	private PathPlannerPath replanAndGetPath(
			int role,
			Pose2d selfPose,
			Pose2d goalPose,
			PathConstraints constraints,
			int[] replanTicks,
			int[] phaseTicks,
			Map<Integer, LocalADStarAK> pathfinderByRole,
			Map<Integer, PathPlannerPath> pathByRole,
			Map<Integer, Pose2d> lastGoalByRole) {
		LocalADStarAK pathfinder = pathfinderByRole.computeIfAbsent(role, key -> new LocalADStarAK());
		PathPlannerPath cachedPath = pathByRole.get(role);
		Pose2d lastGoal = lastGoalByRole.get(role);
		boolean scheduleReplan = shouldReplanForRole(role, replanTicks, phaseTicks);
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
					constraints,
					new GoalEndState(0.0, Rotation2d.fromRadians(0.0)));
			if (newPath != null) {
				pathByRole.put(role, newPath);
				lastGoalByRole.put(role, goalPose);
				cachedPath = newPath;
			}
		}
		return cachedPath;
	} // End replanAndGetPath

	/**
	 * Clears pathfind, follow, and two-path-cycle caches for one extra role. Does not change {@link #lastBehaviorByRole}
	 * or dashboard selection.
	 */
	private void clearTransientExtraMotionCaches(int role) {
		defensePathByRole.remove(role);
		defenseLastGoalByRole.remove(role);
		aggressivePathByRole.remove(role);
		aggressiveLastGoalByRole.remove(role);
		aggressiveBackingOffByRole.remove(role);
		aggressiveBackoffGoalByRole.remove(role);
		aggressiveBackoffStartTickByRole.remove(role);
		aggressiveRamContactTickByRole.remove(role);
		twoPathCyclePhaseByRole.remove(role);
		twoPathCycleFollowPointsByRole.remove(role);
		twoPathCycleFollowIndexByRole.remove(role);
		twoPathCyclePathfinderByRole.remove(role);
		twoPathCyclePathByRole.remove(role);
		twoPathCycleLastGoalByRole.remove(role);
		twoPathCycleLastLaunchTickByRole.remove(role);
		twoPathCyclePathfindLegStartTickByRole.remove(role);
	} // End clearTransientExtraMotionCaches

	/**
	 * Updates {@link #lastBehaviorByRole} and clears motion caches for {@code role} when its selected behavior string
	 * changes.
	 */
	private void resetRoleStateOnBehaviorChange(int role, String selectedBehavior) {
		String previousBehavior = lastBehaviorByRole.get(role);
		if (selectedBehavior.equals(previousBehavior)) {
			return;
		}
		lastBehaviorByRole.put(role, selectedBehavior);
		clearTransientExtraMotionCaches(role);
	} // End resetRoleStateOnBehaviorChange

	/** Computes a 3m backoff point away from the tracked target. */
	private static Pose2d computeAggressiveBackoffGoal(
			Pose2d selfPose,
			Pose2d trackedRobotPose,
			boolean defenderIsRedAlliance) {
		double deltaX = selfPose.getX() - trackedRobotPose.getX();
		double deltaY = selfPose.getY() - trackedRobotPose.getY();
		double distance = Math.hypot(deltaX, deltaY);
		double unitX;
		double unitY;
		if (distance > 1.0e-6) {
			unitX = deltaX / distance;
			unitY = deltaY / distance;
		} else {
			unitX = defenderIsRedAlliance ? kDefenseAggressiveFallbackBackoffXSign : -kDefenseAggressiveFallbackBackoffXSign;
			unitY = 0.0;
		}
		double goalX = selfPose.getX() + unitX * kDefenseAggressiveBackoffDistanceMeters;
		double goalY = selfPose.getY() + unitY * kDefenseAggressiveBackoffDistanceMeters;
		double clampedX = MathUtil.clamp(goalX, 0.25, FieldConstants.FIELD_LENGTH_M - 0.25);
		double clampedY = MathUtil.clamp(goalY, 0.25, FieldConstants.FIELD_WIDTH_M - 0.25);
		return new Pose2d(clampedX, clampedY, selfPose.getRotation());
	} // End computeAggressiveBackoffGoal

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

	/**
	 * Drives field-centrically toward a pose target using independent X/Y proportional gains and a shared linear clamp.
	 * Zero rotation output; callers that want heading control should use
	 * {@link #driveTwoPathCycleFieldCentric(SimFullFieldExtraRobot, Pose2d, Pose2d, double)} instead.
	 */
	private static void driveTowardPoseProportional(
			SimFullFieldExtraRobot extraRobot,
			Pose2d selfPose,
			Pose2d targetPose,
			double kxP,
			double kyP,
			double maxLinearMetersPerSec) {
		double xErrorMeters = targetPose.getX() - selfPose.getX();
		double yErrorMeters = targetPose.getY() - selfPose.getY();
		double vxMetersPerSec = MathUtil.clamp(xErrorMeters * kxP, -maxLinearMetersPerSec, maxLinearMetersPerSec);
		double vyMetersPerSec = MathUtil.clamp(yErrorMeters * kyP, -maxLinearMetersPerSec, maxLinearMetersPerSec);
		extraRobot.drive.driveFieldCentric(vxMetersPerSec, vyMetersPerSec, 0.0);
	} // End driveTowardPoseProportional

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

	/**
	 * Drops pathfind/follow state and forces {@link ExtraSimTwoPathCyclePhase#SCORE_HUB}. Caller must only invoke when
	 * alliance zone, hub shift, and carried fuel gate all passed in {@link #runTwoPathCycleSim}.
	 */
	private void enterTwoPathCycleScoreHubFromTravel(int role, SimFullFieldExtraRobot extraRobot) {
		extraRobot.drive.stop();
		twoPathCyclePathfindLegStartTickByRole.remove(role);
		clearTwoPathCycleNav(role);
		twoPathCycleFollowPointsByRole.remove(role);
		twoPathCycleFollowIndexByRole.remove(role);
		twoPathCyclePathfinderByRole.remove(role);
		twoPathCycleLastLaunchTickByRole.remove(role);
		twoPathCyclePhaseByRole.put(role, ExtraSimTwoPathCyclePhase.SCORE_HUB);
	} // End enterTwoPathCycleScoreHubFromTravel

	/**
	 * Picks {@link ExtraSimTwoPathCyclePhase#PATHFIND_TO_PATHONE} or {@link ExtraSimTwoPathCyclePhase#PATHFIND_TO_PATHTWO}
	 * from whichever path holonomic start is closer in translation (ties favor path one).
	 */
	private static ExtraSimTwoPathCyclePhase pathfindPhaseForCloserHolonomicStart(
			Pose2d robotPose, PathPlannerPath pathOne, PathPlannerPath pathTwo) {
		Pose2d startOne = pathOne.getStartingHolonomicPose().orElse(new Pose2d());
		Pose2d startTwo = pathTwo.getStartingHolonomicPose().orElse(new Pose2d());
		double d1 = robotPose.getTranslation().getDistance(startOne.getTranslation());
		double d2 = robotPose.getTranslation().getDistance(startTwo.getTranslation());
		return d1 <= d2 ? ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHONE : ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHTWO;
	} // End pathfindPhaseForCloserHolonomicStart

	/**
	 * Clears follow/nav state and sets pathfind phase from whichever holonomic path start is closer to the robot.
	 */
	private void twoPathCycleBeginPathfindFromCloserHolonomicStart(
			int role, SimFullFieldExtraRobot extraRobot, PathPlannerPath pathOne, PathPlannerPath pathTwo) {
		twoPathCyclePathfindLegStartTickByRole.remove(role);
		clearTwoPathCycleNav(role);
		twoPathCycleFollowPointsByRole.remove(role);
		twoPathCycleFollowIndexByRole.remove(role);
		twoPathCyclePathfinderByRole.remove(role);
		twoPathCycleLastLaunchTickByRole.remove(role);
		Pose2d pose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		twoPathCyclePhaseByRole.put(role, pathfindPhaseForCloserHolonomicStart(pose, pathOne, pathTwo));
	} // End twoPathCycleBeginPathfindFromCloserHolonomicStart

	/**
	 * Leaves {@link ExtraSimTwoPathCyclePhase#SCORE_HUB} when alliance zone or hub shift gate fails; resumes pathfind
	 * toward the nearer holonomic start.
	 */
	private void exitTwoPathCycleScoreHubToTravel(
			int role, SimFullFieldExtraRobot extraRobot, PathPlannerPath pathOne, PathPlannerPath pathTwo) {
		extraRobot.drive.stop();
		twoPathCycleBeginPathfindFromCloserHolonomicStart(role, extraRobot, pathOne, pathTwo);
	} // End exitTwoPathCycleScoreHubToTravel

	/**
	 * Loads bump PathPlanner paths from deploy once; sets {@link #bumpPathsLoadError} on failure.
	 */
	private static void ensureBumpCyclePathsLoaded() {
		if (bumpPathsLoadAttempted) {
			return;
		}
		synchronized (SimFullFieldExtraBehaviourSim.class) {
			if (bumpPathsLoadAttempted) {
				return;
			}
			bumpPathsLoadAttempted = true;
			try {
				bumpPathOneBlueAuthoring = PathPlannerPath.fromPathFile(BUMP_CYCLE_PATH_ONE_DEPLOY_STEM);
				bumpPathTwoBlueAuthoring = PathPlannerPath.fromPathFile(BUMP_CYCLE_PATH_TWO_DEPLOY_STEM);
				bumpPathOneRedAuthoring = bumpPathOneBlueAuthoring.flipPath();
				bumpPathTwoRedAuthoring = bumpPathTwoBlueAuthoring.flipPath();
			} catch (Exception ex) {
				bumpPathsLoadError = ex.getMessage();
				DriverStation.reportError("SimFullFieldExtra bump paths: " + ex.getMessage(), false);
			}
		}
	} // End ensureBumpCyclePathsLoaded

	/**
	 * @param redAlliance true for red alliance field frame
	 * @return cached bump path one, or null if paths never loaded
	 */
	private static PathPlannerPath bumpDeployPathOneForAlliance(boolean redAlliance) {
		return redAlliance ? bumpPathOneRedAuthoring : bumpPathOneBlueAuthoring;
	} // End bumpDeployPathOneForAlliance

	/**
	 * @param redAlliance true for red alliance field frame
	 * @return cached bump path two, or null if paths never loaded
	 */
	private static PathPlannerPath bumpDeployPathTwoForAlliance(boolean redAlliance) {
		return redAlliance ? bumpPathTwoRedAuthoring : bumpPathTwoBlueAuthoring;
	} // End bumpDeployPathTwoForAlliance

	/**
	 * Runs one tick of {@link #OPTION_CYCLE_BUMP} using the deployed bump paths as {@code pathOne} / {@code pathTwo}.
	 */
	private void runBumpCycleSim(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled) {
		ensureBumpCyclePathsLoaded();
		boolean red = roleIsRedAlliance(role);
		PathPlannerPath pathOne = bumpDeployPathOneForAlliance(red);
		PathPlannerPath pathTwo = bumpDeployPathTwoForAlliance(red);
		if (pathOne == null || pathTwo == null) {
			if (bumpPathsLoadError != null) {
				Logger.recordOutput("SimFullFieldExtra/" + role + "/BumpCycle/LoadError", bumpPathsLoadError);
			}
			extraRobot.drive.stop();
			return;
		}
		runTwoPathCycleSim(role, extraRobot, fuelSim, fuelSimEnabled, pathOne, pathTwo, "BumpCycle");
	} // End runBumpCycleSim

	/**
	 * Loads trench PathPlanner paths from deploy once; sets {@link #trenchPathsLoadError} on failure.
	 */
	private static void ensureTrenchCyclePathsLoaded() {
		if (trenchPathsLoadAttempted) {
			return;
		}
		synchronized (SimFullFieldExtraBehaviourSim.class) {
			if (trenchPathsLoadAttempted) {
				return;
			}
			trenchPathsLoadAttempted = true;
			try {
				trenchPathOneBlueAuthoring = PathPlannerPath.fromPathFile(TRENCH_CYCLE_PATH_ONE_DEPLOY_STEM);
				trenchPathTwoBlueAuthoring = PathPlannerPath.fromPathFile(TRENCH_CYCLE_PATH_TWO_DEPLOY_STEM);
				trenchPathOneRedAuthoring = trenchPathOneBlueAuthoring.flipPath();
				trenchPathTwoRedAuthoring = trenchPathTwoBlueAuthoring.flipPath();
			} catch (Exception ex) {
				trenchPathsLoadError = ex.getMessage();
				DriverStation.reportError("SimFullFieldExtra trench paths: " + ex.getMessage(), false);
			}
		}
	} // End ensureTrenchCyclePathsLoaded

	/**
	 * @param redAlliance true for red alliance field frame
	 * @return cached trench path one, or null if paths never loaded
	 */
	private static PathPlannerPath trenchDeployPathOneForAlliance(boolean redAlliance) {
		return redAlliance ? trenchPathOneRedAuthoring : trenchPathOneBlueAuthoring;
	} // End trenchDeployPathOneForAlliance

	/**
	 * @param redAlliance true for red alliance field frame
	 * @return cached trench path two, or null if paths never loaded
	 */
	private static PathPlannerPath trenchDeployPathTwoForAlliance(boolean redAlliance) {
		return redAlliance ? trenchPathTwoRedAuthoring : trenchPathTwoBlueAuthoring;
	} // End trenchDeployPathTwoForAlliance

	/**
	 * Runs one tick of {@link #OPTION_CYCLE_TRENCH} using {@code AllianceWallSweep} and {@code NeutralZoneSweep} as
	 * {@code pathOne} / {@code pathTwo}.
	 */
	private void runTrenchCycleSim(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled) {
		ensureTrenchCyclePathsLoaded();
		boolean red = roleIsRedAlliance(role);
		PathPlannerPath pathOne = trenchDeployPathOneForAlliance(red);
		PathPlannerPath pathTwo = trenchDeployPathTwoForAlliance(red);
		if (pathOne == null || pathTwo == null) {
			if (trenchPathsLoadError != null) {
				Logger.recordOutput("SimFullFieldExtra/" + role + "/TrenchCycle/LoadError", trenchPathsLoadError);
			}
			extraRobot.drive.stop();
			return;
		}
		runTwoPathCycleSim(role, extraRobot, fuelSim, fuelSimEnabled, pathOne, pathTwo, "TrenchCycle");
	} // End runTrenchCycleSim

	/**
	 * Runs one tick of a two-leg cycle: pathfind to each holonomic path start, follow authored samples, then hub
	 * scoring when allowed. Cold start, pathfind timeout, and resume-after-travel pick the nearer holonomic start
	 * between {@code pathOne} and {@code pathTwo}.
	 *
	 * @param telemetryName logger subfolder under {@code SimFullFieldExtra/{role}/}
	 */
	private void runTwoPathCycleSim(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled,
			PathPlannerPath pathOne,
			PathPlannerPath pathTwo,
			String telemetryName) {
		boolean red = roleIsRedAlliance(role);
		ExtraSimTwoPathCyclePhase phase = twoPathCyclePhaseByRole.get(role);
		if (phase == null) {
			Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
			phase = pathfindPhaseForCloserHolonomicStart(selfPose, pathOne, pathTwo);
			twoPathCyclePhaseByRole.put(role, phase);
		}
		Pose2d poseForScoreHubGate = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		boolean inAllianceZoneForScoreHub =
				AllianceUtil.isInAllianceZone(poseForScoreHubGate.getX(), red);
		boolean theirHubShift = HubShiftUtil.getOfficialShiftInfoForAlliance(red).active();
		int carriedFuelForScoreHubGate = 0;
		if (fuelSimEnabled && fuelSim != null && extraRobot.fuelRobotIndex >= 0) {
			carriedFuelForScoreHubGate = fuelSim.getCarriedFuelCount(extraRobot.fuelRobotIndex);
		}
		boolean hasCarriedFuelForScoreHubGate = carriedFuelForScoreHubGate > 0;
		boolean scoreHubGate = inAllianceZoneForScoreHub && theirHubShift && hasCarriedFuelForScoreHubGate;
		// Do not abort an in-progress authored spline follow; score hub is entered after follow completes or from pathfind.
		boolean onSplineFollowLeg =
				phase == ExtraSimTwoPathCyclePhase.FOLLOW_PATHONE
						|| phase == ExtraSimTwoPathCyclePhase.FOLLOW_PATHTWO;
		if (scoreHubGate && phase != ExtraSimTwoPathCyclePhase.SCORE_HUB && !onSplineFollowLeg) {
			enterTwoPathCycleScoreHubFromTravel(role, extraRobot);
			phase = ExtraSimTwoPathCyclePhase.SCORE_HUB;
		} else if (!scoreHubGate && phase == ExtraSimTwoPathCyclePhase.SCORE_HUB) {
			exitTwoPathCycleScoreHubToTravel(role, extraRobot, pathOne, pathTwo);
			phase = twoPathCyclePhaseByRole.get(role);
		}
		Logger.recordOutput("SimFullFieldExtra/" + role + "/" + telemetryName + "/Phase", phase.name());

		switch (phase) {
			case PATHFIND_TO_PATHONE:
				if (pathfindTowardHolonomicPathStart(role, extraRobot, pathOne)) {
					twoPathCyclePathfindLegStartTickByRole.remove(role);
					clearTwoPathCycleNav(role);
					beginTwoPathCycleFollow(role, pathOne);
					twoPathCyclePhaseByRole.put(role, ExtraSimTwoPathCyclePhase.FOLLOW_PATHONE);
				} else {
					twoPathCyclePathfindLegStartTickByRole.putIfAbsent(role, behaviorTickCounter);
					int legStart = twoPathCyclePathfindLegStartTickByRole.get(role);
					if (behaviorTickCounter - legStart >= kTwoPathCyclePathfindStartTimeoutTicks) {
						reevaluateTwoPathCyclePathfindPhaseFromCloserHolonomicStart(role, extraRobot, pathOne, pathTwo);
					}
				}
				break;
			case FOLLOW_PATHONE:
				if (advanceTwoPathCycleFollow(extraRobot, role)) {
					clearTwoPathCycleNav(role);
					twoPathCyclePathfindLegStartTickByRole.remove(role);
					twoPathCyclePhaseByRole.put(role, ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHTWO);
				}
				break;
			case PATHFIND_TO_PATHTWO:
				if (pathfindTowardHolonomicPathStart(role, extraRobot, pathTwo)) {
					twoPathCyclePathfindLegStartTickByRole.remove(role);
					clearTwoPathCycleNav(role);
					beginTwoPathCycleFollow(role, pathTwo);
					twoPathCyclePhaseByRole.put(role, ExtraSimTwoPathCyclePhase.FOLLOW_PATHTWO);
				} else {
					twoPathCyclePathfindLegStartTickByRole.putIfAbsent(role, behaviorTickCounter);
					int legStart = twoPathCyclePathfindLegStartTickByRole.get(role);
					if (behaviorTickCounter - legStart >= kTwoPathCyclePathfindStartTimeoutTicks) {
						reevaluateTwoPathCyclePathfindPhaseFromCloserHolonomicStart(role, extraRobot, pathOne, pathTwo);
					}
				}
				break;
			case FOLLOW_PATHTWO:
				if (advanceTwoPathCycleFollow(extraRobot, role)) {
					if (scoreHubGate) {
						enterTwoPathCycleScoreHubFromTravel(role, extraRobot);
					} else {
						extraRobot.drive.stop();
						twoPathCycleBeginPathfindFromCloserHolonomicStart(role, extraRobot, pathOne, pathTwo);
					}
				}
				break;
			case SCORE_HUB:
				runTwoPathCycleHubScoringTick(role, extraRobot, fuelSim, fuelSimEnabled, red);
				break;
			default:
				extraRobot.drive.stop();
				break;
		}
	} // End runTwoPathCycleSim

	/**
	 * After {@link #kTwoPathCyclePathfindStartTimeoutTicks} without reaching holonomic start, clears nav cache and sets
	 * pathfind phase from whichever holonomic start is closer (same rule as cold start).
	 */
	private void reevaluateTwoPathCyclePathfindPhaseFromCloserHolonomicStart(
			int role, SimFullFieldExtraRobot extraRobot, PathPlannerPath pathOne, PathPlannerPath pathTwo) {
		clearTwoPathCycleNav(role);
		twoPathCyclePathfindLegStartTickByRole.remove(role);
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		twoPathCyclePhaseByRole.put(role, pathfindPhaseForCloserHolonomicStart(selfPose, pathOne, pathTwo));
	} // End reevaluateTwoPathCyclePathfindPhaseFromCloserHolonomicStart

	/** Drops cached AD-star path segments for this role so the next pathfind leg replans cleanly. */
	private void clearTwoPathCycleNav(int role) {
		twoPathCyclePathByRole.remove(role);
		twoPathCycleLastGoalByRole.remove(role);
	} // End clearTwoPathCycleNav

	/**
	 * Drives toward the holonomic start pose of an authored path using the two-path-cycle navgrid maps.
	 *
	 * @return true when the robot translation is within {@link #kTwoPathCyclePathStartArriveMeters} of the start
	 */
	private boolean pathfindTowardHolonomicPathStart(int role, SimFullFieldExtraRobot extraRobot, PathPlannerPath authoredPath) {
		Pose2d goalPose = authoredPath.getStartingHolonomicPose().orElse(new Pose2d());
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		PathPlannerPath navPath = replanAndGetPath(
				role,
				selfPose,
				goalPose,
				kTwoPathCyclePathConstraints,
				kPrimeReplanTicks,
				kPrimePhaseTicks,
				twoPathCyclePathfinderByRole,
				twoPathCyclePathByRole,
				twoPathCycleLastGoalByRole);
		Pose2d driveTargetPose = goalPose;
		if (navPath != null) {
			driveTargetPose = choosePathTargetPose(navPath, selfPose, goalPose);
		}
		driveTwoPathCycleFieldCentric(extraRobot, selfPose, driveTargetPose, goalPose.getRotation().getRadians());
		return selfPose.getTranslation().getDistance(goalPose.getTranslation()) <= kTwoPathCyclePathStartArriveMeters;
	} // End pathfindTowardHolonomicPathStart

	/**
	 * Copies authored path samples into follow state and resets the follow index to the first point.
	 *
	 * @param role role key for per-role follow storage
	 * @param authoredPath PathPlanner path whose samples are followed in field frame
	 */
	private void beginTwoPathCycleFollow(int role, PathPlannerPath authoredPath) {
		List<Translation2d> points = new ArrayList<>();
		for (PathPoint pathPoint : authoredPath.getAllPathPoints()) {
			points.add(pathPoint.position);
		}
		twoPathCycleFollowPointsByRole.put(role, points);
		twoPathCycleFollowIndexByRole.put(role, 0);
	} // End beginTwoPathCycleFollow

	/**
	 * Steers toward the current follow sample; advances the index when inside tolerance.
	 *
	 * @return true when the last sample is reached within tolerance, or when there are no samples
	 */
	private boolean advanceTwoPathCycleFollow(SimFullFieldExtraRobot extraRobot, int role) {
		List<Translation2d> points = twoPathCycleFollowPointsByRole.get(role);
		if (points == null || points.isEmpty()) {
			return true;
		}
		int index = twoPathCycleFollowIndexByRole.getOrDefault(role, 0);
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		Translation2d target = points.get(Math.min(index, points.size() - 1));
		double distance = selfPose.getTranslation().getDistance(target);
		if (distance < kTwoPathCycleFollowWaypointToleranceMeters) {
			if (index >= points.size() - 1) {
				return true;
			}
			twoPathCycleFollowIndexByRole.put(role, index + 1);
			return false;
		}
		double headingRad = Math.atan2(target.getY() - selfPose.getY(), target.getX() - selfPose.getX());
		driveTwoPathCycleFieldCentric(
				extraRobot,
				selfPose,
				new Pose2d(target, Rotation2d.fromRadians(headingRad)),
				headingRad);
		return false;
	} // End advanceTwoPathCycleFollow

	/**
	 * Field-centric P drive toward {@code targetPose} while tracking {@code headingGoalRad}.
	 */
	private void driveTwoPathCycleFieldCentric(
			SimFullFieldExtraRobot extraRobot,
			Pose2d selfPose,
			Pose2d targetPose,
			double headingGoalRad) {
		double xErrorMeters = targetPose.getX() - selfPose.getX();
		double yErrorMeters = targetPose.getY() - selfPose.getY();
		double vxMetersPerSec = MathUtil.clamp(
				xErrorMeters * kTwoPathCycleFollowLinearP,
				-kTwoPathCycleFollowMaxLinearMetersPerSec,
				kTwoPathCycleFollowMaxLinearMetersPerSec);
		double vyMetersPerSec = MathUtil.clamp(
				yErrorMeters * kTwoPathCycleFollowLinearP,
				-kTwoPathCycleFollowMaxLinearMetersPerSec,
				kTwoPathCycleFollowMaxLinearMetersPerSec);
		double headingErrorRad = MathUtil.angleModulus(headingGoalRad - selfPose.getRotation().getRadians());
		double omegaRadPerSec = MathUtil.clamp(
				headingErrorRad * kTwoPathCycleFollowOmegaP,
				-kTwoPathCycleFollowMaxOmegaRadPerSec,
				kTwoPathCycleFollowMaxOmegaRadPerSec);
		extraRobot.drive.driveFieldCentric(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
	} // End driveTwoPathCycleFieldCentric

	/**
	 * Aims the whole robot like a fixed forward turret and launches one fuel when facing, timing, and fuel state allow.
	 * Hood and exit speed follow {@link ShooterCalculator#iterativeMovingShotFromFunnelClearance} (same path as
	 * {@link frc.robot.commands.ShooterCommands#setShooterTarget} with hood enabled), without {@link
	 * frc.robot.subsystems.shooter.hood.HoodConstants} clamping.
	 *
	 * @param redAlliance selects red vs blue funnel-top hub target
	 */
	private void runTwoPathCycleHubScoringTick(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled,
			boolean redAlliance) {
		if (!fuelSimEnabled || fuelSim == null || extraRobot.fuelRobotIndex < 0) {
			extraRobot.drive.stop();
			return;
		}
		if (fuelSim.getCarriedFuelCount(extraRobot.fuelRobotIndex) <= 0) {
			extraRobot.drive.stop();
			return;
		}
		Pose2d pose = extraRobot.drive.getPose();
		ChassisSpeeds fieldSpeeds = extraRobot.drive.getFieldRelativeChassisSpeeds();
		Translation3d target3d =
				redAlliance ? FieldConstants.RED_FUNNEL_TOP_CENTER_3D : FieldConstants.BLUE_FUNNEL_TOP_CENTER_3D;

		double phaseDelaySec = ShooterConstants.kPhaseDelaySec;
		Pose2d estimatedPose =
				new Pose2d(
						pose.getTranslation()
								.plus(
										new Translation2d(
												fieldSpeeds.vxMetersPerSecond * phaseDelaySec,
												fieldSpeeds.vyMetersPerSecond * phaseDelaySec)),
						pose.getRotation()
								.plus(
										Rotation2d.fromRadians(
												fieldSpeeds.omegaRadiansPerSecond * phaseDelaySec)));

		ShooterCalculator.ShotData shot =
				ShooterCalculator.iterativeMovingShotFromFunnelClearance(
						estimatedPose,
						fieldSpeeds,
						target3d,
						ShooterConstants.kLookaheadIterations);

		double turretYawRobotRad =
				ShooterCalculator.calculateAzimuthAngle(estimatedPose, shot.getTarget(), 0.0).in(Radians);

		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		double headingGoalRad = selfPose.getRotation().getRadians() + turretYawRobotRad;
		driveTwoPathCycleFieldCentric(
				extraRobot,
				selfPose,
				new Pose2d(selfPose.getTranslation(), Rotation2d.fromRadians(headingGoalRad)),
				headingGoalRad);

		if (Math.abs(turretYawRobotRad) > kTwoPathCycleScoreFacingToleranceRad) {
			return;
		}
		int lastLaunchTick = twoPathCycleLastLaunchTickByRole.getOrDefault(role, -1_000_000);
		if (behaviorTickCounter - lastLaunchTick < kTwoPathCycleScoreLaunchIntervalTicks) {
			return;
		}

		double exitVelMps = shot.getExitVelocity().in(MetersPerSecond);
		double flywheelSurfaceSpeedMps =
				exitVelMps
						/ ShooterConstants.kFlywheelSurfaceDivider
						* ShooterConstants.kExitVelocityCompensationMultiplier;
		double ballExitVelMps =
				flywheelSurfaceSpeedMps
						* ShooterConstants.kFlywheelSurfaceDivider
						* ShooterConstants.kSimFlywheelToFuelExitVelocityEfficiency;

		double fuelSimElevationRad = Math.PI / 2.0 - shot.getHoodAngle().in(Radians);
		fuelSim.launchFuel(
				extraRobot.fuelRobotIndex,
				MetersPerSecond.of(ballExitVelMps),
				Radians.of(fuelSimElevationRad),
				Radians.of(turretYawRobotRad),
				ShooterConstants.robotToTurret);
		twoPathCycleLastLaunchTickByRole.put(role, behaviorTickCounter);
	} // End runTwoPathCycleHubScoringTick

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
} // End SimFullFieldExtraBehaviourSim
