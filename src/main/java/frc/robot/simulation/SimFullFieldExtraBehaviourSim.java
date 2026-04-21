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
 * SIM-only behavior chooser manager for full-field extra robots. Primary and Second-Sim are excluded because they are
 * human controlled.
 *
 * <p>Cycle behaviors {@value #OPTION_CYCLE_BUMP} and {@value #OPTION_CYCLE_TRENCH} share the same two-path PathPlanner
 * sim loop (pathfind to each holonomic start, follow, optional hub scoring).
 */
public final class SimFullFieldExtraBehaviourSim {

	public static final String OPTION_DO_NOTHING = "Do Nothing";
	public static final String OPTION_DEFENSE_BLOCK = "Defense (Block)";
	public static final String OPTION_DEFENSE_AGGRESSIVE = "Defense (Aggressive)";
	/** Bump loop: alliance-wall leg then neutral-to-alliance leg plus hub scoring. */
	public static final String OPTION_CYCLE_BUMP = "Cycle (Bump)";
	/** Trench loop: {@code AllianceWallSweep} then {@code NeutralZoneSweep} plus hub scoring. */
	public static final String OPTION_CYCLE_TRENCH = "Cycle (Trench)";

	// ===== Shared pathfinder schedule =====
	/** Prime replan periods per role (different primes prevent collisions). */
	private static final int[] kPrimeReplanTicks = {23, 19, 17, 13, 11};
	/** Role-specific phase offsets for the shared replan schedule. */
	private static final int[] kPrimePhaseTicks = {0, 3, 5, 7, 11};

	// ===== Defense Block tuning =====
	/** Proportional gain to hold X on the trench-neutral block line. */
	private static final double kDefenseBlockPathXP = 5.0;
	/** Proportional gain to match defender Y to the tracked robot Y. */
	private static final double kDefenseBlockPathYP = 4.2;
	/** Linear speed clamp for defense-block steering. */
	private static final double kDefenseBlockPathMaxLinearMetersPerSec = 2.5;
	/** Extra stand-off from trench neutral edge so defenders stay just outside the trench lane. */
	private static final double kDefenseBlockNeutralSideOffsetMeters = 0.6;
	/** If target moves more than this, force immediate replan. */
	private static final double kDefenseBlockTargetReplanDistanceMeters = 0.25;
	/** If robot strays this far from cached path start, force immediate replan. */
	private static final double kDefenseBlockStartReplanDistanceMeters = 0.35;
	/** Waypoint tolerance to advance to next point. */
	private static final double kDefenseBlockWaypointToleranceMeters = 0.35;
	/** Path constraints used by defense block pathfinder. */
	private static final PathConstraints kDefenseBlockPathConstraints =
			new PathConstraints(5.0, 3.0, 2.5 * Math.PI, 3.0 * Math.PI);

	// ===== Defense Aggressive tuning =====
	/** Proportional gain for aggressive chase/backoff X control. */
	private static final double kDefenseAggressivePathXP = 10.0;
	/** Proportional gain for aggressive chase/backoff Y control. */
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
	/** Backoff abort timeout so a stuck backoff resumes chase (5.0s @ 20ms loop). */
	private static final int kDefenseAggressiveBackoffTimeoutTicks = 250;
	/** Separation after contact latch that cancels countdown and resumes chase. */
	private static final double kDefenseAggressiveContactReleaseDistanceMeters = 2.2;
	/** Position tolerance to finish backoff and start next ram. */
	private static final double kDefenseAggressiveBackoffArriveMeters = 0.2;
	/** Fallback backoff heading when target and robot overlap. */
	private static final double kDefenseAggressiveFallbackBackoffXSign = -1.0;

	// ===== Two-path cycle tuning =====
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
	/** Minimum sim ticks between hub launch attempts (20 ms loop, so ~50/value launches per second). */
	private static final int kTwoPathCycleScoreLaunchIntervalTicks = 5;
	/** Max |heading error| (rad) before a hub launch is allowed. */
	private static final double kTwoPathCycleScoreFacingToleranceRad = 0.14;
	/** Sim ticks without reaching holonomic start before re-picking the closer-start leg (2.0 s @ 20 ms loop). */
	private static final int kTwoPathCyclePathfindStartTimeoutTicks = 100;

	// ===== Cycle deploy path sets =====
	/** Immutable deploy stems + lazily-loaded blue/red authoring for one two-path cycle. */
	private static final class CyclePathSet {
		final String telemetryName;
		final String pathOneStem;
		final String pathTwoStem;
		PathPlannerPath pathOneBlue;
		PathPlannerPath pathTwoBlue;
		PathPlannerPath pathOneRed;
		PathPlannerPath pathTwoRed;
		boolean loadAttempted;
		String loadError;

		CyclePathSet(String telemetryName, String pathOneStem, String pathTwoStem) {
			this.telemetryName = telemetryName;
			this.pathOneStem = pathOneStem;
			this.pathTwoStem = pathTwoStem;
		} // End CyclePathSet Constructor

		PathPlannerPath pathOne(boolean red) {
			return red ? pathOneRed : pathOneBlue;
		} // End pathOne

		PathPlannerPath pathTwo(boolean red) {
			return red ? pathTwoRed : pathTwoBlue;
		} // End pathTwo
	} // End CyclePathSet

	private static final CyclePathSet BUMP_CYCLE_PATHS = new CyclePathSet(
			"BumpCycle", "Bump-AllianceToNeutral-Left", "Bump-NeutralToAlliance-Right");
	private static final CyclePathSet TRENCH_CYCLE_PATHS = new CyclePathSet(
			"TrenchCycle", "AllianceWallSweep", "NeutralZoneSweep");

	// ===== Role layout =====
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
			OPTION_DO_NOTHING,         // Blue-2
			OPTION_DEFENSE_BLOCK,      // Blue-3
			OPTION_DEFENSE_BLOCK,      // Red-1
			OPTION_DEFENSE_AGGRESSIVE, // Red-2
			OPTION_DO_NOTHING          // Red-3
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

	// ===== Per-role state =====
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

	// ===== Public API =====

	/** Publishes one behavior chooser per extra role on SmartDashboard. */
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

	/** Returns the selected behavior for {@code role}, falling back to its default when nothing is published yet. */
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
		return ntValue.isEmpty() ? EXTRA_ROLE_DEFAULT_BEHAVIOR[index] : ntValue;
	} // End selectedBehaviorForRole

	/** Resets extra behavior chooser selections when the shared sim reset button is pressed. */
	public void pollResetToDefaults(boolean simMode) {
		if (!simMode || !SmartDashboard.getBoolean("SimStartingPose/ResetToDefaults", false)) {
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
		int[] rolesForLayout = EXTRAS_ROLES_BY_LAYOUT[layoutKey(secondSimEnabled, secondSimRedAlliance)];
		for (int poolIdx = 0; poolIdx < rolesForLayout.length && poolIdx < extraRobotsByPool.length; poolIdx++) {
			SimFullFieldExtraRobot extraRobot = extraRobotsByPool[poolIdx];
			if (extraRobot != null) {
				consumer.accept(rolesForLayout[poolIdx], extraRobot);
			}
		}
	} // End forEachActiveExtra

	/**
	 * Updates all extra robot behaviors for one sim tick.
	 *
	 * <p>Red defenders track Primary. Blue defenders track Second-Sim when it is on red, otherwise Red-2.
	 *
	 * <p>When {@code teleopEnabled} is false, active extras stop and all extra motion caches clear so the next enable
	 * cold-starts pathfind instead of resuming a follow leg.
	 *
	 * @param fuelSim registered fuel simulation instance, or null when fuel sim is off
	 * @param fuelSimEnabled when false, cycle hub scoring treats carried fuel count as zero
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
				extraRobotsByPool, extrasEnabled, secondSimEnabled, secondSimRedAlliance,
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
			dispatchBehavior(
					role, extraRobot, selectedBehavior,
					primaryPose, secondSimPose, red2Pose,
					secondSimEnabled, secondSimRedAlliance,
					fuelSim, fuelSimEnabled);
		}
	} // End updateExtraRobotBehaviors

	// ===== Dispatch =====

	/** Routes one role/tick to the correct behavior runner; stops drive for unknown or {@value #OPTION_DO_NOTHING}. */
	private void dispatchBehavior(
			int role,
			SimFullFieldExtraRobot extraRobot,
			String selectedBehavior,
			Pose2d primaryPose,
			Pose2d secondSimPose,
			Pose2d red2Pose,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance,
			FuelSim fuelSim,
			boolean fuelSimEnabled) {
		if (OPTION_DEFENSE_BLOCK.equals(selectedBehavior) || OPTION_DEFENSE_AGGRESSIVE.equals(selectedBehavior)) {
			Pose2d targetPose = defenseTrackedRobotPose(
					role, primaryPose, secondSimPose, red2Pose, secondSimEnabled, secondSimRedAlliance);
			if (targetPose == null) {
				extraRobot.drive.stop();
				return;
			}
			boolean red = roleIsRedAlliance(role);
			if (OPTION_DEFENSE_BLOCK.equals(selectedBehavior)) {
				runDefenseBlockPathfind(role, extraRobot, red, targetPose);
			} else {
				runDefenseAggressivePathfind(role, extraRobot, red, targetPose);
			}
			return;
		}
		if (OPTION_CYCLE_BUMP.equals(selectedBehavior)) {
			runCycleSim(role, extraRobot, fuelSim, fuelSimEnabled, BUMP_CYCLE_PATHS);
			return;
		}
		if (OPTION_CYCLE_TRENCH.equals(selectedBehavior)) {
			runCycleSim(role, extraRobot, fuelSim, fuelSimEnabled, TRENCH_CYCLE_PATHS);
			return;
		}
		extraRobot.drive.stop();
	} // End dispatchBehavior

	/** 0 = no Second-Sim, 1 = Second-Sim on Blue, 2 = Second-Sim on Red. */
	private static int layoutKey(boolean secondSimEnabled, boolean secondSimRedAlliance) {
		if (!secondSimEnabled) {
			return 0;
		}
		return secondSimRedAlliance ? 2 : 1;
	} // End layoutKey

	/** Returns the tracked target pose for defense (red tracks primary; blue tracks Second-Sim on red, else Red-2). */
	private static Pose2d defenseTrackedRobotPose(
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
	} // End defenseTrackedRobotPose

	// ===== Defense Block =====

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
				role, selfPose, goalPose, kDefenseBlockPathConstraints,
				defensePathfinderByRole, defensePathByRole, defenseLastGoalByRole);
		Pose2d driveTargetPose = cachedPath != null ? choosePathTargetPose(cachedPath, selfPose, goalPose) : goalPose;
		driveTowardPoseProportional(
				extraRobot, selfPose, driveTargetPose,
				kDefenseBlockPathXP, kDefenseBlockPathYP, kDefenseBlockPathMaxLinearMetersPerSec);
	} // End runDefenseBlockPathfind

	// ===== Defense Aggressive =====

	/** Runs aggressive defense by looping between ram and timed backoff. */
	private void runDefenseAggressivePathfind(
			int role,
			SimFullFieldExtraRobot extraRobot,
			boolean defenderIsRedAlliance,
			Pose2d trackedRobotPose) {
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		Pose2d goalPose = aggressiveBackingOffByRole.getOrDefault(role, false)
				? resolveAggressiveBackoffGoal(role, selfPose, trackedRobotPose, defenderIsRedAlliance)
				: resolveAggressiveChaseGoal(role, selfPose, trackedRobotPose, defenderIsRedAlliance);

		PathPlannerPath path = replanAndGetPath(
				role, selfPose, goalPose, kDefenseAggressivePathConstraints,
				aggressivePathfinderByRole, aggressivePathByRole, aggressiveLastGoalByRole);
		Pose2d driveTargetPose = path != null ? choosePathTargetPose(path, selfPose, goalPose) : goalPose;
		driveTowardPoseProportional(
				extraRobot, selfPose, driveTargetPose,
				kDefenseAggressivePathXP, kDefenseAggressivePathYP, kDefenseAggressivePathMaxLinearMetersPerSec);
	} // End runDefenseAggressivePathfind

	/** Chase branch: latches contact on close approach, switches to backoff after the delay, returns current chase goal. */
	private Pose2d resolveAggressiveChaseGoal(
			int role, Pose2d selfPose, Pose2d trackedRobotPose, boolean defenderIsRedAlliance) {
		double distanceToTarget = selfPose.getTranslation().getDistance(trackedRobotPose.getTranslation());
		int contactTick = aggressiveRamContactTickByRole.getOrDefault(role, -1);

		if (contactTick < 0 && distanceToTarget <= kDefenseAggressiveRamSwitchDistanceMeters) {
			contactTick = behaviorTickCounter;
			aggressiveRamContactTickByRole.put(role, contactTick);
		}
		if (contactTick >= 0 && distanceToTarget > kDefenseAggressiveContactReleaseDistanceMeters) {
			aggressiveRamContactTickByRole.remove(role);
			contactTick = -1;
		}
		if (contactTick >= 0 && behaviorTickCounter - contactTick >= kDefenseAggressiveBackoffDelayTicks) {
			Pose2d backoffGoal = computeAggressiveBackoffGoal(selfPose, trackedRobotPose, defenderIsRedAlliance);
			aggressiveBackoffGoalByRole.put(role, backoffGoal);
			aggressiveBackingOffByRole.put(role, true);
			aggressiveBackoffStartTickByRole.put(role, behaviorTickCounter);
			aggressiveRamContactTickByRole.remove(role);
			return backoffGoal;
		}
		return trackedRobotPose;
	} // End resolveAggressiveChaseGoal

	/** Backoff branch: returns cached backoff goal or flips back to chase when arrived or timed out. */
	private Pose2d resolveAggressiveBackoffGoal(
			int role, Pose2d selfPose, Pose2d trackedRobotPose, boolean defenderIsRedAlliance) {
		Pose2d backoffGoal = aggressiveBackoffGoalByRole.getOrDefault(
				role, computeAggressiveBackoffGoal(selfPose, trackedRobotPose, defenderIsRedAlliance));
		double distanceToBackoff = selfPose.getTranslation().getDistance(backoffGoal.getTranslation());
		int backoffStartTick = aggressiveBackoffStartTickByRole.getOrDefault(role, behaviorTickCounter);
		boolean backoffTimedOut = behaviorTickCounter - backoffStartTick >= kDefenseAggressiveBackoffTimeoutTicks;
		boolean backoffArrived = distanceToBackoff <= kDefenseAggressiveBackoffArriveMeters;
		if (backoffArrived || backoffTimedOut) {
			aggressiveBackingOffByRole.put(role, false);
			aggressiveBackoffGoalByRole.remove(role);
			aggressiveBackoffStartTickByRole.remove(role);
			aggressiveRamContactTickByRole.remove(role);
			return trackedRobotPose;
		}
		return backoffGoal;
	} // End resolveAggressiveBackoffGoal

	/** Computes a {@link #kDefenseAggressiveBackoffDistanceMeters} point directly away from the tracked target. */
	private static Pose2d computeAggressiveBackoffGoal(
			Pose2d selfPose, Pose2d trackedRobotPose, boolean defenderIsRedAlliance) {
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

	// ===== Shared navgrid replan + lookahead =====

	/** True when this tick is the role's turn on the prime-tick replan schedule. */
	private boolean shouldReplanForRole(int role) {
		int idx = indexForRole(role);
		if (idx < 0 || idx >= kPrimeReplanTicks.length || idx >= kPrimePhaseTicks.length) {
			return true;
		}
		int period = kPrimeReplanTicks[idx];
		int phase = kPrimePhaseTicks[idx] % period;
		return behaviorTickCounter % period == phase;
	} // End shouldReplanForRole

	/** Replans when schedule/goal/start thresholds require it, then returns cached/new path (null if never planned). */
	private PathPlannerPath replanAndGetPath(
			int role,
			Pose2d selfPose,
			Pose2d goalPose,
			PathConstraints constraints,
			Map<Integer, LocalADStarAK> pathfinderByRole,
			Map<Integer, PathPlannerPath> pathByRole,
			Map<Integer, Pose2d> lastGoalByRole) {
		LocalADStarAK pathfinder = pathfinderByRole.computeIfAbsent(role, key -> new LocalADStarAK());
		PathPlannerPath cachedPath = pathByRole.get(role);
		Pose2d lastGoal = lastGoalByRole.get(role);
		boolean targetMoved = lastGoal == null
				|| lastGoal.getTranslation().getDistance(goalPose.getTranslation()) > kDefenseBlockTargetReplanDistanceMeters;
		List<PathPoint> cachedPoints = cachedPath != null ? cachedPath.getAllPathPoints() : null;
		boolean robotMovedFromStart = cachedPoints == null
				|| cachedPoints.isEmpty()
				|| cachedPoints.get(0).position.getDistance(selfPose.getTranslation()) > kDefenseBlockStartReplanDistanceMeters;
		if (shouldReplanForRole(role) || targetMoved || robotMovedFromStart) {
			pathfinder.setStartPosition(selfPose.getTranslation());
			pathfinder.setGoalPosition(goalPose.getTranslation());
			PathPlannerPath newPath = pathfinder.getCurrentPath(
					constraints, new GoalEndState(0.0, Rotation2d.fromRadians(0.0)));
			if (newPath != null) {
				pathByRole.put(role, newPath);
				lastGoalByRole.put(role, goalPose);
				cachedPath = newPath;
			}
		}
		return cachedPath;
	} // End replanAndGetPath

	/** Selects a short lookahead waypoint on the current path. */
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

	/** Field-centric P drive to a pose with independent X/Y gains, shared linear clamp, and zero rotation output. */
	private static void driveTowardPoseProportional(
			SimFullFieldExtraRobot extraRobot,
			Pose2d selfPose,
			Pose2d targetPose,
			double kxP,
			double kyP,
			double maxLinearMetersPerSec) {
		double vxMetersPerSec = MathUtil.clamp(
				(targetPose.getX() - selfPose.getX()) * kxP, -maxLinearMetersPerSec, maxLinearMetersPerSec);
		double vyMetersPerSec = MathUtil.clamp(
				(targetPose.getY() - selfPose.getY()) * kyP, -maxLinearMetersPerSec, maxLinearMetersPerSec);
		extraRobot.drive.driveFieldCentric(vxMetersPerSec, vyMetersPerSec, 0.0);
	} // End driveTowardPoseProportional

	// ===== Shared alliance / field helpers =====

	/** Returns the neutral-zone-side X of the opposing alliance trench, including a small stand-off offset. */
	private static double trenchNeutralSideXForOpposingAlliance(boolean defenderIsRedAlliance) {
		double blueNeutralSideX = FieldConstants.TRENCH_BUMP_X_M
				+ (FieldConstants.TRENCH_BUMP_LENGTH_M / 2.0)
				+ kDefenseBlockNeutralSideOffsetMeters;
		return defenderIsRedAlliance ? blueNeutralSideX : FieldConstants.FIELD_LENGTH_M - blueNeutralSideX;
	} // End trenchNeutralSideXForOpposingAlliance

	/** True when the fixed extra role belongs to red alliance. */
	private static boolean roleIsRedAlliance(int role) {
		return role == SimStartingPoseFullFieldSim.ROLE_RED_1
				|| role == SimStartingPoseFullFieldSim.ROLE_RED_2
				|| role == SimStartingPoseFullFieldSim.ROLE_RED_3;
	} // End roleIsRedAlliance

	/** Returns carried fuel count for {@code extraRobot}, or 0 when fuel sim is off or the robot has no fuel index. */
	private static int carriedFuelForExtra(SimFullFieldExtraRobot extraRobot, FuelSim fuelSim, boolean fuelSimEnabled) {
		if (!fuelSimEnabled || fuelSim == null || extraRobot.fuelRobotIndex < 0) {
			return 0;
		}
		return fuelSim.getCarriedFuelCount(extraRobot.fuelRobotIndex);
	} // End carriedFuelForExtra

	// ===== Cache reset =====

	/** Clears pathfind, follow, and two-path-cycle caches for one extra role; keeps dashboard selection. */
	private void clearTransientExtraMotionCaches(int role) {
		defensePathByRole.remove(role);
		defenseLastGoalByRole.remove(role);
		aggressivePathByRole.remove(role);
		aggressiveLastGoalByRole.remove(role);
		aggressiveBackingOffByRole.remove(role);
		aggressiveBackoffGoalByRole.remove(role);
		aggressiveBackoffStartTickByRole.remove(role);
		aggressiveRamContactTickByRole.remove(role);
		clearTwoPathCycleTravelState(role);
		twoPathCyclePhaseByRole.remove(role);
	} // End clearTransientExtraMotionCaches

	/** Updates {@link #lastBehaviorByRole} and clears motion caches when the selected behavior changes. */
	private void resetRoleStateOnBehaviorChange(int role, String selectedBehavior) {
		if (selectedBehavior.equals(lastBehaviorByRole.get(role))) {
			return;
		}
		lastBehaviorByRole.put(role, selectedBehavior);
		clearTransientExtraMotionCaches(role);
	} // End resetRoleStateOnBehaviorChange

	// ===== Cycle entry =====

	/** Loads {@code set}'s blue + red (flipped) authoring once; records load error instead of throwing. */
	private static void ensureCyclePathsLoaded(CyclePathSet set) {
		if (set.loadAttempted) {
			return;
		}
		synchronized (SimFullFieldExtraBehaviourSim.class) {
			if (set.loadAttempted) {
				return;
			}
			set.loadAttempted = true;
			try {
				set.pathOneBlue = PathPlannerPath.fromPathFile(set.pathOneStem);
				set.pathTwoBlue = PathPlannerPath.fromPathFile(set.pathTwoStem);
				set.pathOneRed = set.pathOneBlue.flipPath();
				set.pathTwoRed = set.pathTwoBlue.flipPath();
			} catch (Exception ex) {
				set.loadError = ex.getMessage();
				DriverStation.reportError("SimFullFieldExtra " + set.telemetryName + " paths: " + ex.getMessage(), false);
			}
		}
	} // End ensureCyclePathsLoaded

	/** Runs one tick of a cycle behavior after resolving {@code set}'s alliance-correct paths. */
	private void runCycleSim(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled,
			CyclePathSet set) {
		ensureCyclePathsLoaded(set);
		boolean red = roleIsRedAlliance(role);
		PathPlannerPath pathOne = set.pathOne(red);
		PathPlannerPath pathTwo = set.pathTwo(red);
		if (pathOne == null || pathTwo == null) {
			if (set.loadError != null) {
				Logger.recordOutput("SimFullFieldExtra/" + role + "/" + set.telemetryName + "/LoadError", set.loadError);
			}
			extraRobot.drive.stop();
			return;
		}
		runTwoPathCycleSim(role, extraRobot, fuelSim, fuelSimEnabled, pathOne, pathTwo, set.telemetryName);
	} // End runCycleSim

	// ===== Two-path cycle FSM =====

	/**
	 * Runs one tick of a two-leg cycle: pathfind to each holonomic start, follow authored samples, then hub scoring
	 * when allowed. Cold start, pathfind timeout, and resume-after-travel all pick whichever holonomic start is closer.
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
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();

		ExtraSimTwoPathCyclePhase phase = twoPathCyclePhaseByRole.computeIfAbsent(
				role, key -> pathfindPhaseForCloserHolonomicStart(selfPose, pathOne, pathTwo));

		boolean scoreHubGate = AllianceUtil.isInAllianceZone(selfPose.getX(), red)
				&& HubShiftUtil.getOfficialShiftInfoForAlliance(red).active()
				&& carriedFuelForExtra(extraRobot, fuelSim, fuelSimEnabled) > 0;
		boolean onSplineFollowLeg = phase == ExtraSimTwoPathCyclePhase.FOLLOW_PATHONE
				|| phase == ExtraSimTwoPathCyclePhase.FOLLOW_PATHTWO;

		// Score-hub gate flips in/out, except never aborting an in-progress spline follow.
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
				runTwoPathCyclePathfindTick(
						role, extraRobot, pathOne, pathOne, pathTwo, ExtraSimTwoPathCyclePhase.FOLLOW_PATHONE);
				break;
			case FOLLOW_PATHONE:
				if (advanceTwoPathCycleFollow(extraRobot, role)) {
					clearTwoPathCycleNav(role);
					twoPathCyclePathfindLegStartTickByRole.remove(role);
					twoPathCyclePhaseByRole.put(role, ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHTWO);
				}
				break;
			case PATHFIND_TO_PATHTWO:
				runTwoPathCyclePathfindTick(
						role, extraRobot, pathTwo, pathOne, pathTwo, ExtraSimTwoPathCyclePhase.FOLLOW_PATHTWO);
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
	 * Drives toward {@code authoredPath}'s holonomic start; on arrive begins follow and advances to {@code nextPhase};
	 * on pathfind timeout re-picks the closer-start leg.
	 */
	private void runTwoPathCyclePathfindTick(
			int role,
			SimFullFieldExtraRobot extraRobot,
			PathPlannerPath authoredPath,
			PathPlannerPath pathOne,
			PathPlannerPath pathTwo,
			ExtraSimTwoPathCyclePhase nextPhase) {
		if (pathfindTowardHolonomicPathStart(role, extraRobot, authoredPath)) {
			twoPathCyclePathfindLegStartTickByRole.remove(role);
			clearTwoPathCycleNav(role);
			beginTwoPathCycleFollow(role, authoredPath);
			twoPathCyclePhaseByRole.put(role, nextPhase);
			return;
		}
		twoPathCyclePathfindLegStartTickByRole.putIfAbsent(role, behaviorTickCounter);
		int legStart = twoPathCyclePathfindLegStartTickByRole.get(role);
		if (behaviorTickCounter - legStart >= kTwoPathCyclePathfindStartTimeoutTicks) {
			reevaluateTwoPathCyclePathfindPhaseFromCloserHolonomicStart(role, extraRobot, pathOne, pathTwo);
		}
	} // End runTwoPathCyclePathfindTick

	/** Picks the pathfind phase whose holonomic start is closer in XY (ties favor path one). */
	private static ExtraSimTwoPathCyclePhase pathfindPhaseForCloserHolonomicStart(
			Pose2d robotPose, PathPlannerPath pathOne, PathPlannerPath pathTwo) {
		double d1 = robotPose.getTranslation().getDistance(
				pathOne.getStartingHolonomicPose().orElse(new Pose2d()).getTranslation());
		double d2 = robotPose.getTranslation().getDistance(
				pathTwo.getStartingHolonomicPose().orElse(new Pose2d()).getTranslation());
		return d1 <= d2 ? ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHONE : ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHTWO;
	} // End pathfindPhaseForCloserHolonomicStart

	/** Clears pathfind, follow, and launch-timer state for a two-path cycle role (leaves phase untouched). */
	private void clearTwoPathCycleTravelState(int role) {
		twoPathCyclePathfindLegStartTickByRole.remove(role);
		clearTwoPathCycleNav(role);
		twoPathCycleFollowPointsByRole.remove(role);
		twoPathCycleFollowIndexByRole.remove(role);
		twoPathCyclePathfinderByRole.remove(role);
		twoPathCycleLastLaunchTickByRole.remove(role);
	} // End clearTwoPathCycleTravelState

	/** Drops travel state and enters {@link ExtraSimTwoPathCyclePhase#SCORE_HUB}. */
	private void enterTwoPathCycleScoreHubFromTravel(int role, SimFullFieldExtraRobot extraRobot) {
		extraRobot.drive.stop();
		clearTwoPathCycleTravelState(role);
		twoPathCyclePhaseByRole.put(role, ExtraSimTwoPathCyclePhase.SCORE_HUB);
	} // End enterTwoPathCycleScoreHubFromTravel

	/** Drops travel state and sets pathfind phase toward the closer holonomic start. */
	private void twoPathCycleBeginPathfindFromCloserHolonomicStart(
			int role, SimFullFieldExtraRobot extraRobot, PathPlannerPath pathOne, PathPlannerPath pathTwo) {
		clearTwoPathCycleTravelState(role);
		Pose2d pose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		twoPathCyclePhaseByRole.put(role, pathfindPhaseForCloserHolonomicStart(pose, pathOne, pathTwo));
	} // End twoPathCycleBeginPathfindFromCloserHolonomicStart

	/** Leaves {@link ExtraSimTwoPathCyclePhase#SCORE_HUB} and resumes pathfind toward the closer holonomic start. */
	private void exitTwoPathCycleScoreHubToTravel(
			int role, SimFullFieldExtraRobot extraRobot, PathPlannerPath pathOne, PathPlannerPath pathTwo) {
		extraRobot.drive.stop();
		twoPathCycleBeginPathfindFromCloserHolonomicStart(role, extraRobot, pathOne, pathTwo);
	} // End exitTwoPathCycleScoreHubToTravel

	/**
	 * After pathfind timeout, clears nav cache and sets pathfind phase toward the closer holonomic start (same rule as
	 * cold start).
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
	private boolean pathfindTowardHolonomicPathStart(
			int role, SimFullFieldExtraRobot extraRobot, PathPlannerPath authoredPath) {
		Pose2d goalPose = authoredPath.getStartingHolonomicPose().orElse(new Pose2d());
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		PathPlannerPath navPath = replanAndGetPath(
				role, selfPose, goalPose, kTwoPathCyclePathConstraints,
				twoPathCyclePathfinderByRole, twoPathCyclePathByRole, twoPathCycleLastGoalByRole);
		Pose2d driveTargetPose = navPath != null ? choosePathTargetPose(navPath, selfPose, goalPose) : goalPose;
		driveTwoPathCycleFieldCentric(extraRobot, selfPose, driveTargetPose, goalPose.getRotation().getRadians());
		return selfPose.getTranslation().getDistance(goalPose.getTranslation()) <= kTwoPathCyclePathStartArriveMeters;
	} // End pathfindTowardHolonomicPathStart

	/** Copies authored path samples into follow state and resets the follow index to the first point. */
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
				extraRobot, selfPose, new Pose2d(target, Rotation2d.fromRadians(headingRad)), headingRad);
		return false;
	} // End advanceTwoPathCycleFollow

	/** Field-centric P drive toward {@code targetPose} while tracking {@code headingGoalRad}. */
	private void driveTwoPathCycleFieldCentric(
			SimFullFieldExtraRobot extraRobot,
			Pose2d selfPose,
			Pose2d targetPose,
			double headingGoalRad) {
		double vxMetersPerSec = MathUtil.clamp(
				(targetPose.getX() - selfPose.getX()) * kTwoPathCycleFollowLinearP,
				-kTwoPathCycleFollowMaxLinearMetersPerSec, kTwoPathCycleFollowMaxLinearMetersPerSec);
		double vyMetersPerSec = MathUtil.clamp(
				(targetPose.getY() - selfPose.getY()) * kTwoPathCycleFollowLinearP,
				-kTwoPathCycleFollowMaxLinearMetersPerSec, kTwoPathCycleFollowMaxLinearMetersPerSec);
		double headingErrorRad = MathUtil.angleModulus(headingGoalRad - selfPose.getRotation().getRadians());
		double omegaRadPerSec = MathUtil.clamp(
				headingErrorRad * kTwoPathCycleFollowOmegaP,
				-kTwoPathCycleFollowMaxOmegaRadPerSec, kTwoPathCycleFollowMaxOmegaRadPerSec);
		extraRobot.drive.driveFieldCentric(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
	} // End driveTwoPathCycleFieldCentric

	/**
	 * Aims the whole robot like a fixed forward turret and launches one fuel when facing, timing, and fuel state allow.
	 * Hood and exit speed follow {@link ShooterCalculator#iterativeMovingShotFromFunnelClearance} without
	 * {@link frc.robot.subsystems.shooter.hood.HoodConstants} clamping.
	 *
	 * @param redAlliance selects red vs blue funnel-top hub target
	 */
	private void runTwoPathCycleHubScoringTick(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled,
			boolean redAlliance) {
		if (carriedFuelForExtra(extraRobot, fuelSim, fuelSimEnabled) <= 0) {
			extraRobot.drive.stop();
			return;
		}
		Pose2d pose = extraRobot.drive.getPose();
		ChassisSpeeds fieldSpeeds = extraRobot.drive.getFieldRelativeChassisSpeeds();
		Translation3d target3d =
				redAlliance ? FieldConstants.RED_FUNNEL_TOP_CENTER_3D : FieldConstants.BLUE_FUNNEL_TOP_CENTER_3D;

		Pose2d estimatedPose = estimateLookaheadPose(pose, fieldSpeeds, ShooterConstants.kPhaseDelaySec);
		ShooterCalculator.ShotData shot = ShooterCalculator.iterativeMovingShotFromFunnelClearance(
				estimatedPose, fieldSpeeds, target3d, ShooterConstants.kLookaheadIterations);
		double turretYawRobotRad = ShooterCalculator
				.calculateAzimuthAngle(estimatedPose, shot.getTarget(), 0.0)
				.in(Radians);

		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		double headingGoalRad = selfPose.getRotation().getRadians() + turretYawRobotRad;
		driveTwoPathCycleFieldCentric(
				extraRobot, selfPose,
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
		double ballExitVelMps = exitVelMps
				* ShooterConstants.kExitVelocityCompensationMultiplier
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

	/** Projects {@code pose} forward by {@code phaseDelaySec} using current field-relative chassis speeds. */
	private static Pose2d estimateLookaheadPose(Pose2d pose, ChassisSpeeds fieldSpeeds, double phaseDelaySec) {
		return new Pose2d(
				pose.getTranslation().plus(new Translation2d(
						fieldSpeeds.vxMetersPerSecond * phaseDelaySec,
						fieldSpeeds.vyMetersPerSecond * phaseDelaySec)),
				pose.getRotation().plus(Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond * phaseDelaySec)));
	} // End estimateLookaheadPose

	// ===== Dashboard / role index =====

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
