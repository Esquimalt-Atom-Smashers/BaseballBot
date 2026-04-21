package frc.robot.simulation;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * SIM-only: 7 fixed-key PathPlanner starting-pose choosers, full-field extra Maple robots, apply/reset buttons, and
 * per-alliance duplicate-swap. Every chooser is published ONCE under a permanent dashboard key; role-to-key never
 * changes, which avoids the NT topic churn that follows re-publishing a {@link SendableChooser} under different names.
 * All chooser reads go through NT directly ({@link #selectedStem(int)}) so local force-writes are visible immediately.
 */
public final class SimStartingPoseFullFieldSim {

	/** Builds one extra robot at an off-field park pose (adds to arena, Drive, FuelSim registration inside host). */
	@FunctionalInterface
	public interface ExtraRobotFactory {
		SimFullFieldExtraRobot create(int poolIndex, Pose2d parkPose);
	} // End ExtraRobotFactory

	@FunctionalInterface
	public interface PoseSink {
		void accept(Pose2d pose);
	} // End PoseSink

	// Fixed chooser roles. The numeric index is both the position in internal arrays and the public API index used by
	// callers (e.g. RobotContainer passes ROLE_SECOND_SIM to read the Second-Sim chooser).
	public static final int ROLE_PRIMARY = 0;
	public static final int ROLE_SECOND_SIM = 1;
	public static final int ROLE_BLUE_2 = 2;
	public static final int ROLE_BLUE_3 = 3;
	public static final int ROLE_RED_1 = 4;
	public static final int ROLE_RED_2 = 5;
	public static final int ROLE_RED_3 = 6;

	private static final int NUM_ROLES = 7;

	/** Permanent SmartDashboard subtable name per role (appended to {@code "SimStartingPose/"}). */
	private static final String[] ROLE_DASHBOARD_NAME = {
		"Primary-Robot", "Second-Sim", "Blue-2", "Blue-3", "Red-1", "Red-2", "Red-3",
	};

	/** Default stem per role. Used for initial chooser default and for the reset-to-defaults button. */
	private static final String[] ROLE_DEFAULT_STEM = {
		SimStartingPoseUtil.STEM_RIGHT,  // Primary-Robot
		SimStartingPoseUtil.STEM_CENTER, // Second-Sim
		SimStartingPoseUtil.STEM_CENTER, // Blue-2
		SimStartingPoseUtil.STEM_LEFT,   // Blue-3
		SimStartingPoseUtil.STEM_RIGHT,  // Red-1
		SimStartingPoseUtil.STEM_CENTER, // Red-2
		SimStartingPoseUtil.STEM_LEFT,   // Red-3
	};

	/** Pool-ordered extra roles per layout. Size = number of extras for that layout (5 or 4). */
	private static final int[][] EXTRAS_ROLES_BY_LAYOUT = {
		{ ROLE_BLUE_2, ROLE_BLUE_3, ROLE_RED_1, ROLE_RED_2, ROLE_RED_3 }, // Layout 0: no Second-Sim
		{ ROLE_BLUE_3, ROLE_RED_1, ROLE_RED_2, ROLE_RED_3 },              // Layout 1: Second-Sim on Blue
		{ ROLE_BLUE_2, ROLE_BLUE_3, ROLE_RED_2, ROLE_RED_3 },             // Layout 2: Second-Sim on Red
	};

	private final ExtraRobotFactory extraRobotFactory;

	private List<SendableChooser<String>> choosers;

	/** Baseline stems from last poll tick; seeded on first poll; used to detect real user changes. */
	private final String[] previousSelectedStem = new String[NUM_ROLES];
	private boolean duplicateSwapSeeded = false;

	private boolean extrasDashboardPrev;
	private int extrasLayoutKey = -1;
	private SimFullFieldExtraRobot[] extraPool;

	public SimStartingPoseFullFieldSim(ExtraRobotFactory extraRobotFactory) {
		this.extraRobotFactory = extraRobotFactory;
	} // End SimStartingPoseFullFieldSim Constructor

	/** Builds the 7 fixed-role choosers, publishes each at its permanent dashboard key, and allocates the extras pool. */
	public void init() {
		choosers = new ArrayList<>(NUM_ROLES);
		for (int role = 0; role < NUM_ROLES; role++) {
			SendableChooser<String> chooser = new SendableChooser<>();
			for (String stem : SimStartingPoseUtil.PATH_STEMS) {
				chooser.addOption(stem, stem);
			}
			String defaultStem = ROLE_DEFAULT_STEM[role];
			chooser.setDefaultOption(defaultStem, defaultStem);
			choosers.add(chooser);
			SmartDashboard.putData(dashboardKeyForRole(role), chooser);
		}
		extraPool = new SimFullFieldExtraRobot[5];
	} // End init

	/** True while the full-field extras toggle is on (honored by apply/log paths). */
	public boolean extrasEnabled() {
		return extrasDashboardPrev;
	} // End extrasEnabled

	/** 0 = no Second-Sim, 1 = Second-Sim on Blue, 2 = Second-Sim on Red. */
	public int layoutKey(boolean secondSimEnabled, boolean secondSimRedAlliance) {
		if (!secondSimEnabled) {
			return 0;
		}
		return secondSimRedAlliance ? 2 : 1;
	} // End layoutKey

	/**
	 * Stem currently selected for {@code role}, read straight from the NT {@code selected} entry. Bypasses
	 * {@link SendableChooser#getSelected()} because its internal state is not updated by local NT writes.
	 */
	public String selectedStem(int role) {
		String ntValue = NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(dashboardKeyForRole(role))
				.getEntry("selected")
				.getString("");
		if (ntValue == null || ntValue.isEmpty()) {
			return ROLE_DEFAULT_STEM[role];
		}
		return ntValue;
	} // End selectedStem

	/**
	 * Reset button: turns off the full-field extras toggle, forces the Second-Sim mode chooser to Disable via NT, and
	 * writes each role's default stem to its chooser's NT {@code selected} entry. {@code forceSecondSimModeDisable}
	 * encapsulates the NT write for the Second-Sim mode chooser (provided by the caller that owns that chooser).
	 */
	public void pollResetToDefaults(boolean simMode, Runnable forceSecondSimModeDisable) {
		if (!simMode || choosers == null) {
			return;
		}
		if (!SmartDashboard.getBoolean("SimStartingPose/ResetToDefaults", false)) {
			return;
		}
		SmartDashboard.putBoolean("SimFullFieldExtraRobots/Enabled", false);
		forceSecondSimModeDisable.run();
		for (int role = 0; role < NUM_ROLES; role++) {
			forceChooserSelection(role, ROLE_DEFAULT_STEM[role]);
			previousSelectedStem[role] = ROLE_DEFAULT_STEM[role];
		}
		duplicateSwapSeeded = true;
		SmartDashboard.putBoolean("SimStartingPose/ResetToDefaults", false);
	} // End pollResetToDefaults

	/** Apply button: teleports every enabled robot (primary, second-sim, extras) to its currently selected stem. */
	public void pollApplyButton(
			boolean simMode,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance,
			PoseSink primaryPose,
			PoseSink secondPose) {
		if (!simMode || choosers == null) {
			return;
		}
		if (!SmartDashboard.getBoolean("SimStartingPose/Apply", false)) {
			return;
		}
		applyStartingPosesFromChoosers(secondSimEnabled, secondSimRedAlliance, primaryPose, secondPose);
		SmartDashboard.putBoolean("SimStartingPose/Apply", false);
	} // End pollApplyButton

	/**
	 * Enforces per-alliance uniqueness of starting stems. If the user changes role {@code A} to a stem already held by
	 * another active same-alliance role {@code B}, {@code B}'s selection is swapped to {@code A}'s previous stem.
	 * Cross-alliance duplicates are allowed (stems flip by alliance so those poses are physically distinct).
	 */
	public void pollDuplicateSwap(boolean simMode, boolean secondSimEnabled, boolean secondSimRedAlliance) {
		if (!simMode || choosers == null) {
			return;
		}
		int layoutKey = layoutKey(secondSimEnabled, secondSimRedAlliance);
		boolean extrasOn = extrasDashboardPrev;

		// First tick after construction/reset just records the current state without swapping.
		if (!duplicateSwapSeeded) {
			for (int role = 0; role < NUM_ROLES; role++) {
				previousSelectedStem[role] = selectedStem(role);
			}
			duplicateSwapSeeded = true;
			return;
		}

		// Resolve collisions within each alliance independently.
		boolean[] baselineUpdated = new boolean[NUM_ROLES];
		resolveAllianceCollisions(allianceActiveRoles(layoutKey, extrasOn, false), baselineUpdated);
		resolveAllianceCollisions(allianceActiveRoles(layoutKey, extrasOn, true), baselineUpdated);

		// Refresh baselines for roles we did NOT explicitly swap. Swapped roles kept their correct baselines inline.
		for (int role = 0; role < NUM_ROLES; role++) {
			if (!baselineUpdated[role]) {
				previousSelectedStem[role] = selectedStem(role);
			}
		}
	} // End pollDuplicateSwap

	/**
	 * Within a single alliance's active roles, swaps a colliding peer's selection to the just-changed role's prior stem.
	 * Marks {@code baselineUpdated} for both roles so the baseline snapshot at the end of the tick keeps the explicit
	 * post-swap values and does not overwrite them.
	 */
	private void resolveAllianceCollisions(int[] allianceRoles, boolean[] baselineUpdated) {
		for (int changedRole : allianceRoles) {
			String currentStem = selectedStem(changedRole);
			String previousStem = previousSelectedStem[changedRole];
			if (previousStem == null || currentStem.equals(previousStem)) {
				continue;
			}
			// Find any other active same-alliance role currently holding the new stem and give it the vacated stem.
			for (int peerRole : allianceRoles) {
				if (peerRole == changedRole) {
					continue;
				}
				if (currentStem.equals(selectedStem(peerRole))) {
					forceChooserSelection(peerRole, previousStem);
					previousSelectedStem[peerRole] = previousStem;
					previousSelectedStem[changedRole] = currentStem;
					baselineUpdated[peerRole] = true;
					baselineUpdated[changedRole] = true;
					break;
				}
			}
		}
	} // End resolveAllianceCollisions

	/**
	 * Teleports the primary robot, the second sim robot (if enabled), and each enabled full-field extra to the pose
	 * for its currently-selected stem. No validation: the periodic duplicate swap keeps choosers conflict-free live.
	 */
	public void applyStartingPosesFromChoosers(
			boolean secondSimEnabled,
			boolean secondSimRedAlliance,
			PoseSink primaryPose,
			PoseSink secondPose) {
		primaryPose.accept(SimStartingPoseUtil.poseForStem(selectedStem(ROLE_PRIMARY), false));
		if (secondSimEnabled && secondPose != null) {
			secondPose.accept(SimStartingPoseUtil.poseForStem(selectedStem(ROLE_SECOND_SIM), secondSimRedAlliance));
		}
		if (extrasDashboardPrev) {
			applyFullFieldExtraPosesFromChoosers(secondSimEnabled, secondSimRedAlliance);
		}
	} // End applyStartingPosesFromChoosers

	private void applyFullFieldExtraPosesFromChoosers(boolean secondSimEnabled, boolean secondSimRedAlliance) {
		int layoutKey = layoutKey(secondSimEnabled, secondSimRedAlliance);
		int[] rolesForLayout = EXTRAS_ROLES_BY_LAYOUT[layoutKey];
		for (int poolIdx = 0; poolIdx < rolesForLayout.length; poolIdx++) {
			int role = rolesForLayout[poolIdx];
			applyPoolPose(poolIdx, roleIsRedAlliance(role), selectedStem(role));
		}
	} // End applyFullFieldExtraPosesFromChoosers

	private void applyPoolPose(int poolIdx, boolean redAlliance, String stem) {
		SimFullFieldExtraRobot extraRobot = extraPool[poolIdx];
		if (extraRobot == null) {
			return;
		}
		Pose2d pose = SimStartingPoseUtil.poseForStem(stem, redAlliance);
		extraRobot.driveSimulation.setSimulationWorldPose(pose);
		extraRobot.drive.setPose(pose);
	} // End applyPoolPose

	private static Pose2d parkPoseForPoolIndex(int poolIdx) {
		return new Pose2d(-9.0 - 0.35 * poolIdx, 0.5 * poolIdx, new Rotation2d());
	} // End parkPoseForPoolIndex

	private void parkExtraSlot(int poolIdx) {
		if (extraPool == null || extraPool[poolIdx] == null) {
			return;
		}
		SimFullFieldExtraRobot extraRobot = extraPool[poolIdx];
		Pose2d parkPose = parkPoseForPoolIndex(poolIdx);
		extraRobot.driveSimulation.setSimulationWorldPose(parkPose);
		extraRobot.drive.setPose(parkPose);
		extraRobot.drive.stop();
	} // End parkExtraSlot

	private void parkAllExtras() {
		if (extraPool == null) {
			return;
		}
		for (int poolIndex = 0; poolIndex < extraPool.length; poolIndex++) {
			parkExtraSlot(poolIndex);
		}
	} // End parkAllExtras

	private void ensureExtraAt(int poolIdx, int role) {
		if (extraPool[poolIdx] != null) {
			extraPool[poolIdx].role = role;
			return;
		}
		Pose2d parkPose = parkPoseForPoolIndex(poolIdx);
		SimFullFieldExtraRobot extraRobot = extraRobotFactory.create(poolIdx, parkPose);
		extraRobot.role = role;
		extraPool[poolIdx] = extraRobot;
	} // End ensureExtraAt

	private void ensureExtraBodies(boolean secondSimEnabled, boolean secondSimRedAlliance) {
		int layoutKey = layoutKey(secondSimEnabled, secondSimRedAlliance);
		int[] rolesForLayout = EXTRAS_ROLES_BY_LAYOUT[layoutKey];
		for (int poolIdx = 0; poolIdx < rolesForLayout.length; poolIdx++) {
			ensureExtraAt(poolIdx, rolesForLayout[poolIdx]);
		}
		// Park any pool slots above the active extras count for this layout.
		for (int unusedPoolIdx = rolesForLayout.length; unusedPoolIdx < extraPool.length; unusedPoolIdx++) {
			parkExtraSlot(unusedPoolIdx);
		}
	} // End ensureExtraBodies

	/** Polls the enable toggle; spawns / parks extras on edges and when the layout changes. */
	public void pollFullFieldExtrasDashboard(boolean simMode, boolean secondSimEnabled, boolean secondSimRedAlliance) {
		if (!simMode || extraPool == null) {
			return;
		}
		boolean want = SmartDashboard.getBoolean("SimFullFieldExtraRobots/Enabled", false);
		if (want) {
			int layoutKey = layoutKey(secondSimEnabled, secondSimRedAlliance);
			if (!extrasDashboardPrev || layoutKey != extrasLayoutKey) {
				extrasLayoutKey = layoutKey;
				ensureExtraBodies(secondSimEnabled, secondSimRedAlliance);
				applyFullFieldExtraPosesFromChoosers(secondSimEnabled, secondSimRedAlliance);
			}
		} else {
			extrasLayoutKey = -1;
			if (extrasDashboardPrev) {
				parkAllExtras();
				logExtraPoses(secondSimEnabled, secondSimRedAlliance);
			}
		}
		extrasDashboardPrev = want;
	} // End pollFullFieldExtrasDashboard

	public void logExtraPosesIfEnabled(boolean secondSimEnabled, boolean secondSimRedAlliance) {
		if (extraPool == null || !extrasDashboardPrev) {
			return;
		}
		logExtraPoses(secondSimEnabled, secondSimRedAlliance);
	} // End logExtraPosesIfEnabled

	private void logExtraPoses(boolean secondSimEnabled, boolean secondSimRedAlliance) {
		if (extraPool == null) {
			return;
		}
		int layoutKey = layoutKey(secondSimEnabled, secondSimRedAlliance);
		int[] rolesForLayout = EXTRAS_ROLES_BY_LAYOUT[layoutKey];
		for (int poolIdx = 0; poolIdx < rolesForLayout.length; poolIdx++) {
			SimFullFieldExtraRobot extraRobot = extraPool[poolIdx];
			if (extraRobot == null) {
				continue;
			}
			Pose2d loggedPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
			Logger.recordOutput("FieldSimulation/RobotPosition-" + ROLE_DASHBOARD_NAME[extraRobot.role], loggedPose);
		}
	} // End logExtraPoses

	/** Writes {@code stem} to the {@code selected} NT entry for {@code role}'s SendableChooser subtable. */
	private static void forceChooserSelection(int role, String stem) {
		NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(dashboardKeyForRole(role))
				.getEntry("selected")
				.setString(stem);
	} // End forceChooserSelection

	/**
	 * Re-publishes each starting-pose chooser selected value to its NT {@code selected} entry.
	 *
	 * <p>Used to force chooser-widget rebind on fresh dashboard startup or NT reconnect.
	 */
	public void republishSelectedChoices() {
		for (int role = 0; role < NUM_ROLES; role++) {
			forceChooserSelection(role, selectedStem(role));
		}
	} // End republishSelectedChoices

	/** Permanent SmartDashboard key for {@code role} (e.g. {@code "SimStartingPose/Blue-3"}). */
	private static String dashboardKeyForRole(int role) {
		return "SimStartingPose/" + ROLE_DASHBOARD_NAME[role];
	} // End dashboardKeyForRole

	/** True if {@code role} always belongs to the red alliance (used for extras; Second-Sim alliance depends on layout). */
	private static boolean roleIsRedAlliance(int role) {
		return role == ROLE_RED_1 || role == ROLE_RED_2 || role == ROLE_RED_3;
	} // End roleIsRedAlliance

	/**
	 * Active roles on the requested alliance for the current layout / extras state. Primary is always active on blue;
	 * Second-Sim participates on its configured alliance; extra roles participate only when the extras toggle is on.
	 */
	private static int[] allianceActiveRoles(int layoutKey, boolean extrasEnabled, boolean forRedAlliance) {
		List<Integer> roles = new ArrayList<>(4);
		if (!forRedAlliance) {
			roles.add(ROLE_PRIMARY);
			if (layoutKey == 1) {
				roles.add(ROLE_SECOND_SIM);
			}
			if (extrasEnabled) {
				if (layoutKey == 0 || layoutKey == 2) {
					roles.add(ROLE_BLUE_2);
				}
				roles.add(ROLE_BLUE_3);
			}
		} else {
			if (layoutKey == 2) {
				roles.add(ROLE_SECOND_SIM);
			}
			if (extrasEnabled) {
				if (layoutKey == 0 || layoutKey == 1) {
					roles.add(ROLE_RED_1);
				}
				roles.add(ROLE_RED_2);
				roles.add(ROLE_RED_3);
			}
		}
		int[] out = new int[roles.size()];
		for (int i = 0; i < out.length; i++) {
			out[i] = roles.get(i);
		}
		return out;
	} // End allianceActiveRoles
} // End SimStartingPoseFullFieldSim
