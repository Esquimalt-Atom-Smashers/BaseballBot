package frc.robot.simulation;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Predicate;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
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
	/** Driver control using dedicated controller ports for selected extra roles. */
	public static final String OPTION_HUMAN_CONTROL = "Human Control";

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
	/** Backoff abort timeout so a stuck backoff resumes chase (1.22s @ 20ms loop). */
	private static final int kDefenseAggressiveBackoffTimeoutTicks = 61;
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
	/** Consider follow leg stalled when field speed stays below this while still far from target sample. */
	private static final double kTwoPathCycleFollowStuckSpeedMps = 0.08;
	/** Consecutive low-speed follow ticks required before forcing re-pathfind (1.06 s @ 20 ms loop). */
	private static final int kTwoPathCycleFollowStuckTicks = 53;
	/** Min per-tick distance improvement to count as progress while following an authored sample. */
	private static final double kTwoPathCycleFollowProgressEpsilonMeters = 0.5;
	/** Consecutive no-progress ticks while near another robot before forcing re-pathfind (1.06 s @ 20 ms loop). */
	private static final int kTwoPathCycleFollowNoProgressTicks = 53;
	/** Treat another robot as "in contact range" when centers are within this distance. */
	private static final double kTwoPathCycleFollowRobotContactRangeMeters = 1.0;
	/** Minimum sim ticks between hub launch attempts (20 ms loop, so ~50/value launches per second). */
	private static final int kTwoPathCycleScoreLaunchIntervalTicks = 5;
	/** Max |heading error| (rad) before a hub launch is allowed. */
	private static final double kTwoPathCycleScoreFacingToleranceRad = 0.14;
	/** After losing facing tolerance, keep launching for this many sim ticks once shooting already started (0.62s @ 20ms). */
	private static final int kTwoPathCycleScoreAimLossGraceTicks = 31;
	/** Sim ticks without reaching holonomic start before re-picking the closer-start leg (2.0 s @ 20 ms loop). */
	private static final int kTwoPathCyclePathfindStartTimeoutTicks = 100;
	/** Half-width/half-length (m) of dynamic obstacle AABBs used for robot avoidance in AD*. */
	private static final double kDynamicObstacleHalfExtentMeters = 0.45;
	/** If an obstacle center is within this distance of the planner goal, skip it to avoid goal self-blocking. */
	private static final double kDynamicObstacleGoalExclusionMeters = 0.8;
	/** Extra kitbots shoot out the back of the robot, so rear points at the hub (180 deg from +X forward). */
	private static final double kKitbotRearShooterYawOffsetRad = Math.PI;
	/** Kitbot shooter position relative to robot center. */
	private static final Transform3d kKitbotShooterPos = new Transform3d(new Translation3d(0.3395, 0.0, 0.205), new Rotation3d());
	/** Sentinel meaning "no launch yet"; chosen large-negative so {@code tickCounter - sentinel} clears any interval. */
	private static final int kLastLaunchTickUnset = -1_000_000;

	// ===== Human Control =====
	private static final int kHumanControllerPortBlue2 = 1;
	private static final int kHumanControllerPortBlue3 = 2;
	private static final int kHumanControllerPortRed1 = 3;
	private static final int kHumanControllerPortRed2 = 4;
	private static final int kHumanControllerPortRed3 = 5;

	// ===== Cycle deploy path sets =====
	/** Immutable deploy stems + lazily-loaded blue/red authoring plus cached holonomic starts for one two-path cycle. */
	private static final class CyclePathSet {
		final String telemetryName;
		final String pathOneStem;
		final String pathTwoStem;
		PathPlannerPath pathOneBlue;
		PathPlannerPath pathTwoBlue;
		PathPlannerPath pathOneRed;
		PathPlannerPath pathTwoRed;
		Pose2d pathOneStartBlue;
		Pose2d pathTwoStartBlue;
		Pose2d pathOneStartRed;
		Pose2d pathTwoStartRed;
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

		Pose2d pathOneStart(boolean red) {
			return red ? pathOneStartRed : pathOneStartBlue;
		} // End pathOneStart

		Pose2d pathTwoStart(boolean red) {
			return red ? pathTwoStartRed : pathTwoStartBlue;
		} // End pathTwoStart
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

	/** Cached AD-star pathfinder plus its latest plan output for one behavior+role. */
	private static final class PathPlanCache {
		LocalADStarAK pathfinder;
		PathPlannerPath path;
		Pose2d lastGoal;
	} // End PathPlanCache

	/** All per-role transient state bundled into one record so lookups cost one map hit. */
	private static final class RoleState {
		// Defense block
		final PathPlanCache defensePlan = new PathPlanCache();
		// Defense aggressive
		final PathPlanCache aggressivePlan = new PathPlanCache();
		boolean aggressiveBackingOff;
		Pose2d aggressiveBackoffGoal;
		int aggressiveBackoffStartTick;
		int aggressiveRamContactTick = -1;
		// Behavior change tracking
		String lastBehavior;
		// Latest pose for active role this tick; null when not active.
		Pose2d latestPose;
		// Two-path cycle
		ExtraSimTwoPathCyclePhase twoPathCyclePhase;
		List<Translation2d> twoPathCycleFollowPoints;
		int twoPathCycleFollowIndex;
		int twoPathCycleFollowStuckTicks;
		Double twoPathCycleFollowLastDistance;
		int twoPathCycleFollowNoProgressTicks;
		final PathPlanCache twoPathCyclePlan = new PathPlanCache();
		int twoPathCycleLastLaunchTick = kLastLaunchTickUnset;
		boolean twoPathCycleHasStartedShooting;
		int twoPathCycleScoreAimGraceUntilTick = Integer.MIN_VALUE;
		Integer twoPathCyclePathfindLegStartTick;
	} // End RoleState

	private final Map<Integer, RoleState> roleStateByRole = new HashMap<>();
	private int behaviorTickCounter = 0;
	private Pose2d latestPrimaryPose;
	private Pose2d latestSecondSimPose;
	private final Map<Integer, CommandXboxController> humanControllerByRole = new HashMap<>();
	private final Map<Integer, Integer> humanControllerPortByRole = new HashMap<>();
	private final Map<Integer, Boolean> humanPrevRightBumperByRole = new HashMap<>();
	private final Map<Integer, ProfiledPIDController> humanFaceTargetControllerByRole = new HashMap<>();

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

	/** Result of one authored-path follow tick. */
	private enum TwoPathCycleFollowStatus {
		IN_PROGRESS,
		COMPLETED,
		STUCK
	}

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
			behaviorChooser.addOption(OPTION_HUMAN_CONTROL, OPTION_HUMAN_CONTROL);
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

	/** Returns a role→robot map of active extras for the current layout (only non-null robots included). */
	public Map<Integer, SimFullFieldExtraRobot> activeExtrasByRole(
			SimFullFieldExtraRobot[] extraRobotsByPool,
			boolean extrasEnabled,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance) {
		Map<Integer, SimFullFieldExtraRobot> active = new HashMap<>();
		if (!extrasEnabled || extraRobotsByPool == null) {
			return active;
		}
		int[] rolesForLayout = EXTRAS_ROLES_BY_LAYOUT[layoutKey(secondSimEnabled, secondSimRedAlliance)];
		for (int poolIdx = 0; poolIdx < rolesForLayout.length && poolIdx < extraRobotsByPool.length; poolIdx++) {
			SimFullFieldExtraRobot extraRobot = extraRobotsByPool[poolIdx];
			if (extraRobot != null) {
				active.put(rolesForLayout[poolIdx], extraRobot);
			}
		}
		return active;
	} // End activeExtrasByRole

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
		Map<Integer, SimFullFieldExtraRobot> activeExtraByRole = activeExtrasByRole(
				extraRobotsByPool, extrasEnabled, secondSimEnabled, secondSimRedAlliance);

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
		latestPrimaryPose = primaryPose;
		latestSecondSimPose = secondSimPose;

		clearAllLatestPoses();
		for (Map.Entry<Integer, SimFullFieldExtraRobot> entry : activeExtraByRole.entrySet()) {
			roleState(entry.getKey()).latestPose = entry.getValue().driveSimulation.getSimulatedDriveTrainPose();
		}

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
		if (OPTION_HUMAN_CONTROL.equals(selectedBehavior)) {
			runHumanControlSim(role, extraRobot, fuelSim, fuelSimEnabled, secondSimEnabled);
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

	/** Runs direct human control for selected extra roles using dedicated controller ports. */
	private void runHumanControlSim(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled,
			boolean secondSimEnabled) {
		CommandXboxController controller = humanControllerForRole(role, secondSimEnabled);
		if (controller == null) {
			extraRobot.drive.stop();
			return;
		}

		// Get linear velocity
		Translation2d linearVelocity =
				DriveCommands.getLinearVelocityFromJoysticks(-controller.getLeftX(), -controller.getLeftY());
				
		// Square turbo input for quadratic response (always positive, so just square it)
		double turboInput = controller.getRightTriggerAxis();
		turboInput = turboInput * turboInput;

		// Determine speed limits based on parameter
		double maxLinearSpeed = extraRobot.drive.getMaxLinearSpeedMetersPerSec();
		double maxAngularRate = extraRobot.drive.getMaxAngularSpeedRadPerSec();

		// Apply turbo scaling to linear velocity components
		double velocityX = DriveCommands.scaleAxisWithTurbo(linearVelocity.getY(), turboInput, maxLinearSpeed);  // Forward/backward joystick → field X
		double velocityY = DriveCommands.scaleAxisWithTurbo(linearVelocity.getX(), turboInput, maxLinearSpeed);  // Left/right joystick → field Y

		// Determine rotational rate: use face-target PID if enabled, otherwise use joystick
		boolean faceAndShoot = controller.getHID().getAButton();
		double rotationalRate;
		if (faceAndShoot) {
			autoSelectShootingTargetForExtra(extraRobot, roleIsRedAlliance(role));
			// Calculate target angle and use PID controller to rotate toward it
			rotationalRate = computeOmegaToFaceHubRear(extraRobot.drive, faceTargetControllerForRole(role));
			runTwoPathCycleHubScoringTick(extraRobot, roleState(role), fuelSim, fuelSimEnabled);
		} else {
			// Apply rotation deadband
			double omega = MathUtil.applyDeadband(-controller.getRightX(), Constants.ControllerConstants.CONTROLLER_DEADBAND);
			
			// Square rotation value for more precise control
			omega = Math.copySign(omega * omega, omega);

			// Apply turbo scaling to omega
			rotationalRate = DriveCommands.scaleAxisWithTurbo(omega, turboInput, maxAngularRate);
			
			// Reset PID controller when not using face-target mode
			faceTargetControllerForRole(role).reset(extraRobot.drive.getRotation().getRadians());
		}
		driveFieldRelativeForAlliance(extraRobot.drive, velocityX, velocityY, rotationalRate, roleIsRedAlliance(role));
	} // End runHumanControlSim

	private CommandXboxController humanControllerForRole(int role, boolean secondSimEnabled) {
		Integer port = switch (role) {
			case SimStartingPoseFullFieldSim.ROLE_BLUE_2 -> kHumanControllerPortBlue2;
			case SimStartingPoseFullFieldSim.ROLE_BLUE_3 -> kHumanControllerPortBlue3;
			case SimStartingPoseFullFieldSim.ROLE_RED_1 -> kHumanControllerPortRed1;
			case SimStartingPoseFullFieldSim.ROLE_RED_2 -> kHumanControllerPortRed2;
			case SimStartingPoseFullFieldSim.ROLE_RED_3 -> kHumanControllerPortRed3;
			default -> null;
		};
		if (port == null) {
			return null;
		}
		Integer cachedPort = humanControllerPortByRole.get(role);
		if (cachedPort == null || cachedPort != port) {
			CommandXboxController controller = new CommandXboxController(port);
			humanControllerByRole.put(role, controller);
			humanControllerPortByRole.put(role, port);
			return controller;
		}
		return humanControllerByRole.get(role);
	} // End humanControllerForRole

	private ProfiledPIDController faceTargetControllerForRole(int role) {
		return humanFaceTargetControllerByRole.computeIfAbsent(
				role,
				key -> {
					ProfiledPIDController controller =
							new ProfiledPIDController(
									DriveCommands.getAngleKp(),
									0.0,
									DriveCommands.getAngleKd(),
									new TrapezoidProfile.Constraints(
											DriveCommands.getAngleMaxVelocity(),
											DriveCommands.getAngleMaxAcceleration()));
					controller.enableContinuousInput(-Math.PI, Math.PI);
					return controller;
				});
	} // End faceTargetControllerForRole

	/** Returns omega command to point the Kitbot rear shooter at the hub. */
	private static double computeOmegaToFaceHubRear(Drive drive, ProfiledPIDController faceTargetController) {
		Rotation2d angleFromPivotToHub = ShooterCommands.getFieldAngleToHubFromPivot(drive);
		Rotation2d rearFacingTarget = angleFromPivotToHub.plus(Rotation2d.fromRadians(kKitbotRearShooterYawOffsetRad));
		return faceTargetController.calculate(drive.getRotation().getRadians(), rearFacingTarget.getRadians());
	} // End computeOmegaToFaceHubRear

	/** Field-centric drive with blue-origin alliance flip (red gets +180 deg reference rotation). */
	private static void driveFieldRelativeForAlliance(
			Drive drive, double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec, boolean redAlliance) {
		Rotation2d referenceRotation = redAlliance
				? drive.getRotation().plus(Rotation2d.fromRadians(Math.PI))
				: drive.getRotation();
		drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
				vxMetersPerSec, vyMetersPerSec, omegaRadPerSec, referenceRotation));
	} // End driveFieldRelativeForAlliance

	/** Mirrors ShootWhenReady auto target selection for extras (hub in alliance zone, pass spot otherwise). */
	private static void autoSelectShootingTargetForExtra(SimFullFieldExtraRobot extraRobot, boolean redAlliance) {
		ShooterCommands.registerTargetAllianceSupplier(extraRobot.drive, () -> redAlliance);
		Pose2d pose = extraRobot.drive.getPose();
		double zoneTolerance = ShooterConstants.kAutoSelectShootingTargetAllianceZoneTolerance;
		boolean inAllianceZone = redAlliance
				? pose.getX() > FieldConstants.FIELD_LENGTH_M - FieldConstants.ALLIANCE_ZONE_M - zoneTolerance
				: pose.getX() < FieldConstants.ALLIANCE_ZONE_M + zoneTolerance;
		if (inAllianceZone) {
			ShooterCommands.clearShooterTargetOverride(extraRobot.drive);
			return;
		}
		boolean aboveCenterY = pose.getY() > FieldConstants.FIELD_CENTER_Y_M;
		boolean passLeft = aboveCenterY ^ redAlliance;
		if (passLeft) {
			ShooterCommands.setPassingSpotLeft(extraRobot.drive);
		} else {
			ShooterCommands.setPassingSpotRight(extraRobot.drive);
		}
	} // End autoSelectShootingTargetForExtra

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
				role, selfPose, goalPose, kDefenseBlockPathConstraints, roleState(role).defensePlan);
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
		RoleState state = roleState(role);
		Pose2d goalPose = state.aggressiveBackingOff
				? resolveAggressiveBackoffGoal(state, selfPose, trackedRobotPose, defenderIsRedAlliance)
				: resolveAggressiveChaseGoal(state, selfPose, trackedRobotPose, defenderIsRedAlliance);

		PathPlannerPath path = replanAndGetPath(
				role, selfPose, goalPose, kDefenseAggressivePathConstraints, state.aggressivePlan);
		Pose2d driveTargetPose = path != null ? choosePathTargetPose(path, selfPose, goalPose) : goalPose;
		driveTowardPoseProportional(
				extraRobot, selfPose, driveTargetPose,
				kDefenseAggressivePathXP, kDefenseAggressivePathYP, kDefenseAggressivePathMaxLinearMetersPerSec);
	} // End runDefenseAggressivePathfind

	/** Chase branch: latches contact on close approach, switches to backoff after the delay, returns current chase goal. */
	private Pose2d resolveAggressiveChaseGoal(
			RoleState state, Pose2d selfPose, Pose2d trackedRobotPose, boolean defenderIsRedAlliance) {
		double distanceToTarget = selfPose.getTranslation().getDistance(trackedRobotPose.getTranslation());

		if (state.aggressiveRamContactTick < 0 && distanceToTarget <= kDefenseAggressiveRamSwitchDistanceMeters) {
			state.aggressiveRamContactTick = behaviorTickCounter;
		}
		if (state.aggressiveRamContactTick >= 0 && distanceToTarget > kDefenseAggressiveContactReleaseDistanceMeters) {
			state.aggressiveRamContactTick = -1;
		}
		if (state.aggressiveRamContactTick >= 0
				&& behaviorTickCounter - state.aggressiveRamContactTick >= kDefenseAggressiveBackoffDelayTicks) {
			Pose2d backoffGoal = computeAggressiveBackoffGoal(selfPose, trackedRobotPose, defenderIsRedAlliance);
			state.aggressiveBackoffGoal = backoffGoal;
			state.aggressiveBackingOff = true;
			state.aggressiveBackoffStartTick = behaviorTickCounter;
			state.aggressiveRamContactTick = -1;
			return backoffGoal;
		}
		return trackedRobotPose;
	} // End resolveAggressiveChaseGoal

	/** Backoff branch: returns cached backoff goal or flips back to chase when arrived or timed out. */
	private Pose2d resolveAggressiveBackoffGoal(
			RoleState state, Pose2d selfPose, Pose2d trackedRobotPose, boolean defenderIsRedAlliance) {
		Pose2d backoffGoal = state.aggressiveBackoffGoal != null
				? state.aggressiveBackoffGoal
				: computeAggressiveBackoffGoal(selfPose, trackedRobotPose, defenderIsRedAlliance);
		double distanceToBackoff = selfPose.getTranslation().getDistance(backoffGoal.getTranslation());
		boolean backoffTimedOut =
				behaviorTickCounter - state.aggressiveBackoffStartTick >= kDefenseAggressiveBackoffTimeoutTicks;
		boolean backoffArrived = distanceToBackoff <= kDefenseAggressiveBackoffArriveMeters;
		if (backoffArrived || backoffTimedOut) {
			state.aggressiveBackingOff = false;
			state.aggressiveBackoffGoal = null;
			state.aggressiveBackoffStartTick = 0;
			state.aggressiveRamContactTick = -1;
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
			PathPlanCache cache) {
		if (cache.pathfinder == null) {
			cache.pathfinder = new LocalADStarAK();
		}
		cache.pathfinder.setDynamicObstacles(
				dynamicObstaclesForRole(role, goalPose.getTranslation(), kDynamicObstacleGoalExclusionMeters),
				selfPose.getTranslation());
		PathPlannerPath cachedPath = cache.path;
		Pose2d lastGoal = cache.lastGoal;
		boolean targetMoved = lastGoal == null
				|| lastGoal.getTranslation().getDistance(goalPose.getTranslation()) > kDefenseBlockTargetReplanDistanceMeters;
		List<PathPoint> cachedPoints = cachedPath != null ? cachedPath.getAllPathPoints() : null;
		boolean robotMovedFromStart = cachedPoints == null
				|| cachedPoints.isEmpty()
				|| cachedPoints.get(0).position.getDistance(selfPose.getTranslation()) > kDefenseBlockStartReplanDistanceMeters;
		if (shouldReplanForRole(role) || targetMoved || robotMovedFromStart) {
			cache.pathfinder.setStartPosition(selfPose.getTranslation());
			cache.pathfinder.setGoalPosition(goalPose.getTranslation());
			PathPlannerPath newPath = cache.pathfinder.getCurrentPath(
					constraints, new GoalEndState(0.0, Rotation2d.fromRadians(0.0)));
			if (newPath != null) {
				cache.path = newPath;
				cache.lastGoal = goalPose;
				cachedPath = newPath;
			}
		}
		return cachedPath;
	} // End replanAndGetPath

	/**
	 * Dynamic obstacle list for AD* in world coordinates using primary, second-sim, and other active extras.
	 *
	 * <p>Current role is excluded so a robot never blocks its own start region.
	 */
	private List<Pair<Translation2d, Translation2d>> dynamicObstaclesForRole(
			int role, Translation2d goalTranslation, double goalExclusionMeters) {
		List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();
		forEachOtherActiveRobotPose(role, pose ->
				addDynamicObstacleFromPose(obstacles, pose, goalTranslation, goalExclusionMeters));
		return obstacles;
	} // End dynamicObstaclesForRole

	/** Applies consumer to primary, second-sim, and each active extra pose except {@code excludedRole}. */
	private void forEachOtherActiveRobotPose(int excludedRole, Consumer<Pose2d> consumer) {
		if (latestPrimaryPose != null) {
			consumer.accept(latestPrimaryPose);
		}
		if (latestSecondSimPose != null) {
			consumer.accept(latestSecondSimPose);
		}
		for (Map.Entry<Integer, RoleState> entry : roleStateByRole.entrySet()) {
			if (entry.getKey() == excludedRole) {
				continue;
			}
			Pose2d pose = entry.getValue().latestPose;
			if (pose != null) {
				consumer.accept(pose);
			}
		}
	} // End forEachOtherActiveRobotPose

	/** Short-circuiting variant: returns true as soon as a pose satisfies {@code test}. */
	private boolean anyOtherActiveRobotPose(int excludedRole, Predicate<Pose2d> test) {
		if (latestPrimaryPose != null && test.test(latestPrimaryPose)) {
			return true;
		}
		if (latestSecondSimPose != null && test.test(latestSecondSimPose)) {
			return true;
		}
		for (Map.Entry<Integer, RoleState> entry : roleStateByRole.entrySet()) {
			if (entry.getKey() == excludedRole) {
				continue;
			}
			Pose2d pose = entry.getValue().latestPose;
			if (pose != null && test.test(pose)) {
				return true;
			}
		}
		return false;
	} // End anyOtherActiveRobotPose

	/** Appends an axis-aligned obstacle box centered at {@code pose} using {@link #kDynamicObstacleHalfExtentMeters}. */
	private static void addDynamicObstacleFromPose(
			List<Pair<Translation2d, Translation2d>> obstacles,
			Pose2d pose,
			Translation2d goalTranslation,
			double goalExclusionMeters) {
		if (pose == null) {
			return;
		}
		if (goalTranslation != null
				&& pose.getTranslation().getDistance(goalTranslation) <= goalExclusionMeters) {
			return;
		}
		Translation2d min = new Translation2d(
				pose.getX() - kDynamicObstacleHalfExtentMeters,
				pose.getY() - kDynamicObstacleHalfExtentMeters);
		Translation2d max = new Translation2d(
				pose.getX() + kDynamicObstacleHalfExtentMeters,
				pose.getY() + kDynamicObstacleHalfExtentMeters);
		obstacles.add(new Pair<>(min, max));
	} // End addDynamicObstacleFromPose

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

	// ===== Role state access =====

	/** Lazily creates the state record for {@code role}. */
	private RoleState roleState(int role) {
		return roleStateByRole.computeIfAbsent(role, key -> new RoleState());
	} // End roleState

	/** Clears {@link RoleState#latestPose} for every known role so only active extras are set this tick. */
	private void clearAllLatestPoses() {
		for (RoleState state : roleStateByRole.values()) {
			state.latestPose = null;
		}
	} // End clearAllLatestPoses

	// ===== Cache reset =====

	/** Clears pathfind, follow, and two-path-cycle caches for one extra role; keeps dashboard selection. */
	private void clearTransientExtraMotionCaches(int role) {
		RoleState state = roleStateByRole.get(role);
		if (state == null) {
			return;
		}
		resetPathPlan(state.defensePlan);
		resetPathPlan(state.aggressivePlan);
		state.aggressiveBackingOff = false;
		state.aggressiveBackoffGoal = null;
		state.aggressiveBackoffStartTick = 0;
		state.aggressiveRamContactTick = -1;
		clearTwoPathCycleTravelState(state);
		state.twoPathCyclePhase = null;
		humanPrevRightBumperByRole.remove(role);
		humanControllerPortByRole.remove(role);
	} // End clearTransientExtraMotionCaches

	/** Drops cached plan output on {@code cache}; keeps pathfinder so AD* can resume warm. */
	private static void resetPathPlan(PathPlanCache cache) {
		cache.path = null;
		cache.lastGoal = null;
	} // End resetPathPlan

	/** Updates {@link RoleState#lastBehavior} and clears motion caches when the selected behavior changes. */
	private void resetRoleStateOnBehaviorChange(int role, String selectedBehavior) {
		RoleState state = roleState(role);
		if (selectedBehavior.equals(state.lastBehavior)) {
			return;
		}
		state.lastBehavior = selectedBehavior;
		clearTransientExtraMotionCaches(role);
	} // End resetRoleStateOnBehaviorChange

	// ===== Cycle entry =====

	/** Loads {@code set}'s blue + red (flipped) authoring once and caches holonomic starts; records load error instead of throwing. */
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
				set.pathOneStartBlue = set.pathOneBlue.getStartingHolonomicPose().orElse(new Pose2d());
				set.pathTwoStartBlue = set.pathTwoBlue.getStartingHolonomicPose().orElse(new Pose2d());
				set.pathOneStartRed = set.pathOneRed.getStartingHolonomicPose().orElse(new Pose2d());
				set.pathTwoStartRed = set.pathTwoRed.getStartingHolonomicPose().orElse(new Pose2d());
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
		Pose2d pathOneStart = set.pathOneStart(red);
		Pose2d pathTwoStart = set.pathTwoStart(red);
		runTwoPathCycleSim(
				role, extraRobot, fuelSim, fuelSimEnabled, pathOne, pathTwo, pathOneStart, pathTwoStart, set.telemetryName);
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
			Pose2d pathOneStart,
			Pose2d pathTwoStart,
			String telemetryName) {
		boolean red = roleIsRedAlliance(role);
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		RoleState state = roleState(role);

		if (state.twoPathCyclePhase == null) {
			state.twoPathCyclePhase = pathfindPhaseForCloserHolonomicStart(selfPose, pathOneStart, pathTwoStart);
		}
		ExtraSimTwoPathCyclePhase phase = state.twoPathCyclePhase;

		boolean scoreHubGate = AllianceUtil.isInAllianceZone(selfPose.getX(), red)
				&& HubShiftUtil.getOfficialShiftInfoForAlliance(red).active()
				&& carriedFuelForExtra(extraRobot, fuelSim, fuelSimEnabled) > 0;
		boolean onSplineFollowLeg = phase == ExtraSimTwoPathCyclePhase.FOLLOW_PATHONE
				|| phase == ExtraSimTwoPathCyclePhase.FOLLOW_PATHTWO;

		// Score-hub gate flips in/out, except never aborting an in-progress spline follow.
		if (scoreHubGate && phase != ExtraSimTwoPathCyclePhase.SCORE_HUB && !onSplineFollowLeg) {
			enterTwoPathCycleScoreHubFromTravel(state, extraRobot);
			phase = ExtraSimTwoPathCyclePhase.SCORE_HUB;
		} else if (!scoreHubGate && phase == ExtraSimTwoPathCyclePhase.SCORE_HUB) {
			twoPathCycleBeginPathfindFromCloserHolonomicStart(state, extraRobot, selfPose, pathOneStart, pathTwoStart);
			phase = state.twoPathCyclePhase;
		}
		Logger.recordOutput("SimFullFieldExtra/" + role + "/" + telemetryName + "/Phase", phase.name());

		switch (phase) {
			case PATHFIND_TO_PATHONE:
				runTwoPathCyclePathfindTick(
						role, extraRobot, state, selfPose, pathOne, pathOneStart,
						pathOneStart, pathTwoStart, ExtraSimTwoPathCyclePhase.FOLLOW_PATHONE);
				break;
			case FOLLOW_PATHONE:
				TwoPathCycleFollowStatus followPathOneStatus = advanceTwoPathCycleFollow(role, extraRobot, state, selfPose);
				if (followPathOneStatus == TwoPathCycleFollowStatus.COMPLETED) {
					clearTwoPathCycleNav(state);
					state.twoPathCyclePathfindLegStartTick = null;
					state.twoPathCyclePhase = ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHTWO;
				} else if (followPathOneStatus == TwoPathCycleFollowStatus.STUCK) {
					twoPathCycleBeginPathfindFromCloserHolonomicStart(state, extraRobot, selfPose, pathOneStart, pathTwoStart);
				}
				break;
			case PATHFIND_TO_PATHTWO:
				runTwoPathCyclePathfindTick(
						role, extraRobot, state, selfPose, pathTwo, pathTwoStart,
						pathOneStart, pathTwoStart, ExtraSimTwoPathCyclePhase.FOLLOW_PATHTWO);
				break;
			case FOLLOW_PATHTWO:
				TwoPathCycleFollowStatus followPathTwoStatus = advanceTwoPathCycleFollow(role, extraRobot, state, selfPose);
				if (followPathTwoStatus == TwoPathCycleFollowStatus.COMPLETED) {
					if (scoreHubGate) {
						enterTwoPathCycleScoreHubFromTravel(state, extraRobot);
					} else {
						twoPathCycleBeginPathfindFromCloserHolonomicStart(
								state, extraRobot, selfPose, pathOneStart, pathTwoStart);
					}
				} else if (followPathTwoStatus == TwoPathCycleFollowStatus.STUCK) {
					twoPathCycleBeginPathfindFromCloserHolonomicStart(state, extraRobot, selfPose, pathOneStart, pathTwoStart);
				}
				break;
			case SCORE_HUB:
				autoSelectShootingTargetForExtra(extraRobot, red);
				runTwoPathCycleHubScoringTick(extraRobot, state, fuelSim, fuelSimEnabled);
				break;
			default:
				extraRobot.drive.stop();
				break;
		}
	} // End runTwoPathCycleSim

	/**
	 * Drives toward {@code authoredPathStart}; on arrive begins follow and advances to {@code nextPhase}; on pathfind
	 * timeout re-picks the closer-start leg.
	 */
	private void runTwoPathCyclePathfindTick(
			int role,
			SimFullFieldExtraRobot extraRobot,
			RoleState state,
			Pose2d selfPose,
			PathPlannerPath authoredPath,
			Pose2d authoredPathStart,
			Pose2d pathOneStart,
			Pose2d pathTwoStart,
			ExtraSimTwoPathCyclePhase nextPhase) {
		if (pathfindTowardHolonomicPathStart(role, extraRobot, state, selfPose, authoredPathStart)) {
			state.twoPathCyclePathfindLegStartTick = null;
			clearTwoPathCycleNav(state);
			beginTwoPathCycleFollow(state, authoredPath);
			state.twoPathCyclePhase = nextPhase;
			return;
		}
		if (state.twoPathCyclePathfindLegStartTick == null) {
			state.twoPathCyclePathfindLegStartTick = behaviorTickCounter;
		}
		int legStart = state.twoPathCyclePathfindLegStartTick;
		if (behaviorTickCounter - legStart >= kTwoPathCyclePathfindStartTimeoutTicks) {
			reevaluateTwoPathCyclePathfindPhaseFromCloserHolonomicStart(state, selfPose, pathOneStart, pathTwoStart);
		}
	} // End runTwoPathCyclePathfindTick

	/** Picks the pathfind phase whose holonomic start is closer in XY (ties favor path one). */
	private static ExtraSimTwoPathCyclePhase pathfindPhaseForCloserHolonomicStart(
			Pose2d robotPose, Pose2d pathOneStart, Pose2d pathTwoStart) {
		double d1 = robotPose.getTranslation().getDistance(pathOneStart.getTranslation());
		double d2 = robotPose.getTranslation().getDistance(pathTwoStart.getTranslation());
		return d1 <= d2 ? ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHONE : ExtraSimTwoPathCyclePhase.PATHFIND_TO_PATHTWO;
	} // End pathfindPhaseForCloserHolonomicStart

	/** Clears pathfind, follow, and launch-timer state for a two-path cycle role (leaves phase untouched). */
	private static void clearTwoPathCycleTravelState(RoleState state) {
		state.twoPathCyclePathfindLegStartTick = null;
		clearTwoPathCycleNav(state);
		state.twoPathCycleFollowPoints = null;
		state.twoPathCycleFollowIndex = 0;
		state.twoPathCycleFollowStuckTicks = 0;
		state.twoPathCycleFollowLastDistance = null;
		state.twoPathCycleFollowNoProgressTicks = 0;
		state.twoPathCyclePlan.pathfinder = null;
		state.twoPathCycleLastLaunchTick = kLastLaunchTickUnset;
		state.twoPathCycleHasStartedShooting = false;
		state.twoPathCycleScoreAimGraceUntilTick = Integer.MIN_VALUE;
	} // End clearTwoPathCycleTravelState

	/** Stops drive, drops travel state, and enters {@link ExtraSimTwoPathCyclePhase#SCORE_HUB}. */
	private static void enterTwoPathCycleScoreHubFromTravel(RoleState state, SimFullFieldExtraRobot extraRobot) {
		extraRobot.drive.stop();
		clearTwoPathCycleTravelState(state);
		state.twoPathCyclePhase = ExtraSimTwoPathCyclePhase.SCORE_HUB;
	} // End enterTwoPathCycleScoreHubFromTravel

	/** Stops drive, drops travel state, and sets pathfind phase toward the closer holonomic start. */
	private static void twoPathCycleBeginPathfindFromCloserHolonomicStart(
			RoleState state,
			SimFullFieldExtraRobot extraRobot,
			Pose2d selfPose,
			Pose2d pathOneStart,
			Pose2d pathTwoStart) {
		extraRobot.drive.stop();
		clearTwoPathCycleTravelState(state);
		state.twoPathCyclePhase = pathfindPhaseForCloserHolonomicStart(selfPose, pathOneStart, pathTwoStart);
	} // End twoPathCycleBeginPathfindFromCloserHolonomicStart

	/**
	 * After pathfind timeout, clears nav cache and sets pathfind phase toward the closer holonomic start (same rule as
	 * cold start) without resetting follow or launch state.
	 */
	private static void reevaluateTwoPathCyclePathfindPhaseFromCloserHolonomicStart(
			RoleState state, Pose2d selfPose, Pose2d pathOneStart, Pose2d pathTwoStart) {
		clearTwoPathCycleNav(state);
		state.twoPathCyclePathfindLegStartTick = null;
		state.twoPathCyclePhase = pathfindPhaseForCloserHolonomicStart(selfPose, pathOneStart, pathTwoStart);
	} // End reevaluateTwoPathCyclePathfindPhaseFromCloserHolonomicStart

	/** Drops cached AD-star path segments for this role so the next pathfind leg replans cleanly. */
	private static void clearTwoPathCycleNav(RoleState state) {
		state.twoPathCyclePlan.path = null;
		state.twoPathCyclePlan.lastGoal = null;
	} // End clearTwoPathCycleNav

	/**
	 * Drives toward the holonomic start pose of an authored path using the two-path-cycle navgrid maps.
	 *
	 * @return true when the robot translation is within {@link #kTwoPathCyclePathStartArriveMeters} of the start
	 */
	private boolean pathfindTowardHolonomicPathStart(
			int role,
			SimFullFieldExtraRobot extraRobot,
			RoleState state,
			Pose2d selfPose,
			Pose2d goalPose) {
		PathPlannerPath navPath = replanAndGetPath(
				role, selfPose, goalPose, kTwoPathCyclePathConstraints, state.twoPathCyclePlan);
		Pose2d driveTargetPose = navPath != null ? choosePathTargetPose(navPath, selfPose, goalPose) : goalPose;
		driveTwoPathCycleFieldCentric(extraRobot, selfPose, driveTargetPose, goalPose.getRotation().getRadians());
		return selfPose.getTranslation().getDistance(goalPose.getTranslation()) <= kTwoPathCyclePathStartArriveMeters;
	} // End pathfindTowardHolonomicPathStart

	/** Copies authored path samples into follow state and resets the follow index to the first point. */
	private static void beginTwoPathCycleFollow(RoleState state, PathPlannerPath authoredPath) {
		List<PathPoint> authoredPoints = authoredPath.getAllPathPoints();
		List<Translation2d> points = new ArrayList<>(authoredPoints.size());
		for (PathPoint pathPoint : authoredPoints) {
			points.add(pathPoint.position);
		}
		state.twoPathCycleFollowPoints = points;
		state.twoPathCycleFollowIndex = 0;
		state.twoPathCycleFollowStuckTicks = 0;
		state.twoPathCycleFollowLastDistance = null;
		state.twoPathCycleFollowNoProgressTicks = 0;
	} // End beginTwoPathCycleFollow

	/**
	 * Steers toward the current follow sample; advances the index when inside tolerance.
	 *
	 * @return {@link TwoPathCycleFollowStatus#COMPLETED} when final sample is reached, {@link
	 *         TwoPathCycleFollowStatus#STUCK} after sustained low-speed stall, otherwise {@link
	 *         TwoPathCycleFollowStatus#IN_PROGRESS}
	 */
	private TwoPathCycleFollowStatus advanceTwoPathCycleFollow(
			int role, SimFullFieldExtraRobot extraRobot, RoleState state, Pose2d selfPose) {
		List<Translation2d> points = state.twoPathCycleFollowPoints;
		if (points == null || points.isEmpty()) {
			return TwoPathCycleFollowStatus.COMPLETED;
		}
		int index = state.twoPathCycleFollowIndex;
		Translation2d target = points.get(Math.min(index, points.size() - 1));
		double distance = selfPose.getTranslation().getDistance(target);
		if (distance < kTwoPathCycleFollowWaypointToleranceMeters) {
			state.twoPathCycleFollowStuckTicks = 0;
			state.twoPathCycleFollowNoProgressTicks = 0;
			state.twoPathCycleFollowLastDistance = null;
			if (index >= points.size() - 1) {
				return TwoPathCycleFollowStatus.COMPLETED;
			}
			state.twoPathCycleFollowIndex = index + 1;
			return TwoPathCycleFollowStatus.IN_PROGRESS;
		}

		ChassisSpeeds fieldSpeeds = extraRobot.drive.getFieldRelativeChassisSpeeds();
		double linearSpeedMps = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
		int stuckTicks = linearSpeedMps < kTwoPathCycleFollowStuckSpeedMps ? state.twoPathCycleFollowStuckTicks + 1 : 0;
		state.twoPathCycleFollowStuckTicks = stuckTicks;
		if (stuckTicks >= kTwoPathCycleFollowStuckTicks) {
			state.twoPathCycleFollowStuckTicks = 0;
			return TwoPathCycleFollowStatus.STUCK;
		}

		double previousDistance = state.twoPathCycleFollowLastDistance != null
				? state.twoPathCycleFollowLastDistance : distance;
		boolean madeProgress = previousDistance - distance > kTwoPathCycleFollowProgressEpsilonMeters;
		state.twoPathCycleFollowLastDistance = distance;
		int noProgressTicks = madeProgress ? 0 : state.twoPathCycleFollowNoProgressTicks + 1;
		state.twoPathCycleFollowNoProgressTicks = noProgressTicks;
		if (noProgressTicks >= kTwoPathCycleFollowNoProgressTicks
				&& nearAnotherRobot(role, selfPose, kTwoPathCycleFollowRobotContactRangeMeters)) {
			state.twoPathCycleFollowNoProgressTicks = 0;
			state.twoPathCycleFollowLastDistance = null;
			return TwoPathCycleFollowStatus.STUCK;
		}

		double headingRad = Math.atan2(target.getY() - selfPose.getY(), target.getX() - selfPose.getX());
		driveTwoPathCycleFieldCentric(
				extraRobot, selfPose, new Pose2d(target, Rotation2d.fromRadians(headingRad)), headingRad);
		return TwoPathCycleFollowStatus.IN_PROGRESS;
	} // End advanceTwoPathCycleFollow

	/** True when {@code selfPose} is within {@code contactRangeMeters} of primary/second/any other active extra robot. */
	private boolean nearAnotherRobot(int role, Pose2d selfPose, double contactRangeMeters) {
		Translation2d selfTranslation = selfPose.getTranslation();
		return anyOtherActiveRobotPose(
				role, pose -> selfTranslation.getDistance(pose.getTranslation()) <= contactRangeMeters);
	} // End nearAnotherRobot

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
	 */
	private void runTwoPathCycleHubScoringTick(
			SimFullFieldExtraRobot extraRobot,
			RoleState state,
			FuelSim fuelSim,
			boolean fuelSimEnabled) {
		if (carriedFuelForExtra(extraRobot, fuelSim, fuelSimEnabled) <= 0) {
			extraRobot.drive.stop();
			return;
		}
		// Shooter aims from believed (odometry) pose, not ground-truth sim pose, to match real robot behavior.
		Pose2d pose = extraRobot.drive.getPose();
		ChassisSpeeds fieldSpeeds = extraRobot.drive.getFieldRelativeChassisSpeeds();
		Translation3d target3d = ShooterCommands.getShooterTarget3d(extraRobot.drive);

		Pose2d estimatedPose = estimateLookaheadPose(pose, fieldSpeeds, ShooterConstants.kPhaseDelaySec);
		Pose2d estimatedLaunchPose = launchPointPoseFromRobotPose(estimatedPose, kKitbotShooterPos);
		ShooterCalculator.ShotData shot = ShooterCalculator.iterativeMovingShotFromFunnelClearance(
				estimatedLaunchPose, fieldSpeeds, target3d, ShooterConstants.kLookaheadIterations);
		double turretYawRobotRad = ShooterCalculator
				.calculateAzimuthAngle(estimatedLaunchPose, shot.getTarget(), 0.0)
				.in(Radians);

		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		double rearYawErrorRad = MathUtil.angleModulus(turretYawRobotRad - kKitbotRearShooterYawOffsetRad);
		double headingGoalRad = selfPose.getRotation().getRadians() + rearYawErrorRad;
		driveTwoPathCycleFieldCentric(
				extraRobot, selfPose,
				new Pose2d(selfPose.getTranslation(), Rotation2d.fromRadians(headingGoalRad)),
				headingGoalRad);

		boolean inFacingTolerance = Math.abs(rearYawErrorRad) <= kTwoPathCycleScoreFacingToleranceRad;
		if (inFacingTolerance) {
			state.twoPathCycleScoreAimGraceUntilTick = behaviorTickCounter + kTwoPathCycleScoreAimLossGraceTicks;
		}
		boolean canLaunchForAim = inFacingTolerance
				|| (state.twoPathCycleHasStartedShooting && behaviorTickCounter <= state.twoPathCycleScoreAimGraceUntilTick);
		if (!canLaunchForAim) {
			return;
		}
		if (behaviorTickCounter - state.twoPathCycleLastLaunchTick < kTwoPathCycleScoreLaunchIntervalTicks) {
			return;
		}

		double exitVelMps = shot.getExitVelocity().in(MetersPerSecond);
		double ballExitVelMps = exitVelMps
				* ShooterConstants.kExitVelocityCompensationMultiplier
				* ShooterConstants.kSimFlywheelToFuelExitVelocityEfficiency;
		double fuelSimElevationRad = shot.getHoodAngle().in(Radians);
		// Fixed kitbot shooter: launch direction is always out the rear in robot frame.
		double launchYawRobotRad = kKitbotRearShooterYawOffsetRad;
		fuelSim.launchFuel(
				extraRobot.fuelRobotIndex,
				MetersPerSecond.of(ballExitVelMps),
				Radians.of(fuelSimElevationRad),
				Radians.of(launchYawRobotRad),
				kKitbotShooterPos);
		state.twoPathCycleLastLaunchTick = behaviorTickCounter;
		state.twoPathCycleHasStartedShooting = true;
	} // End runTwoPathCycleHubScoringTick

	/** Projects {@code pose} forward by {@code phaseDelaySec} using current field-relative chassis speeds. */
	private static Pose2d estimateLookaheadPose(Pose2d pose, ChassisSpeeds fieldSpeeds, double phaseDelaySec) {
		return new Pose2d(
				pose.getTranslation().plus(new Translation2d(
						fieldSpeeds.vxMetersPerSecond * phaseDelaySec,
						fieldSpeeds.vyMetersPerSecond * phaseDelaySec)),
				pose.getRotation().plus(Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond * phaseDelaySec)));
	} // End estimateLookaheadPose

	/**
	 * Returns field pose of a robot-relative launch point transform (translation only, pose yaw unchanged).
	 */
	private static Pose2d launchPointPoseFromRobotPose(Pose2d robotPose, Transform3d robotToLaunchPoint) {
		Translation3d launchOffset = robotToLaunchPoint.getTranslation();
		Translation2d launchOffsetXYRobot = new Translation2d(launchOffset.getX(), launchOffset.getY());
		Translation2d launchOffsetXYField = launchOffsetXYRobot.rotateBy(robotPose.getRotation());
		return new Pose2d(robotPose.getTranslation().plus(launchOffsetXYField), robotPose.getRotation());
	} // End launchPointPoseFromRobotPose

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

	/**
	 * Re-publishes each extra behavior chooser selected value to its NT {@code selected} entry.
	 *
	 * <p>Used to force chooser-widget rebind on fresh dashboard startup or NT reconnect.
	 */
	public void republishSelectedChoices() {
		for (int index = 0; index < EXTRA_ROLES.length; index++) {
			forceBehaviorSelectionByIndex(index, selectedBehaviorForRole(EXTRA_ROLES[index]));
		}
	} // End republishSelectedChoices
} // End SimFullFieldExtraBehaviourSim
