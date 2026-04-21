package frc.robot.simulation;

import frc.robot.util.HubShiftUtil;

/**
 * Physics-sim-only team-sign text: uses {@link FuelSim} hub scores and {@link HubShiftUtil} shift
 * timing. Use {@link #formatLineForSimulatedMatchBlue(int)} or {@link #formatLineForSimulatedMatchRed(int)}
 * so the RP fuel field matches the alliance display (blue hub score vs red hub score). Shift R/B
 * letters are field-centric and identical on both lines.
 * {@link frc.robot.Robot} gates latch/window calls to {@code Constants.Mode.SIM} only; this class
 * does not re-check mode.
 */
public final class TeamSignDisplayUtil {

	/** Combined-fuel denominator shown until Energized RP is earned. */
	public static final int kEnergizedFuelRpThreshold = 240;

	/** Combined-fuel denominator shown between Energized and Supercharged RP. */
	public static final int kSuperchargedFuelRpThreshold = 360;

	/** Outcome of comparing red vs blue auto hub fuel; carries the shift-schedule alternation. */
	public enum AutoOutcome {
		RED_WINS(true),
		BLUE_WINS(false),
		TIE(false);

		private final boolean scheduleRedFirst;

		AutoOutcome(boolean scheduleRedFirst) {
			this.scheduleRedFirst = scheduleRedFirst;
		}

		/** True when the red hub is the first single-hub-active side (used for R/B alternation). */
		public boolean scheduleRedFirst() {
			return scheduleRedFirst;
		}
	}

	private static volatile AutoOutcome latchedOutcome = AutoOutcome.TIE;

	private TeamSignDisplayUtil() {}

	/** Compares auto-period fuel deposited in each hub. */
	public static AutoOutcome outcomeFromAutoFuel(int redAutoHubFuel, int blueAutoHubFuel) {
		if (redAutoHubFuel > blueAutoHubFuel) {
			return AutoOutcome.RED_WINS;
		}
		if (blueAutoHubFuel > redAutoHubFuel) {
			return AutoOutcome.BLUE_WINS;
		}
		return AutoOutcome.TIE;
	} // End outcomeFromAutoFuel

	/**
	 * Latches the current {@link FuelSim.Hub} scores as the auto result. Call once at the start of
	 * teleop so the R/B alternation reflects autonomous only.
	 */
	public static void latchAutoHubScoresFromFuelSim() {
		latchedOutcome = outcomeFromAutoFuel(FuelSim.Hub.RED_HUB.getScore(), FuelSim.Hub.BLUE_HUB.getScore());
	} // End latchAutoHubScoresFromFuelSim

	/** Resets the latched auto outcome back to {@link AutoOutcome#TIE}. */
	public static void clearLatchedAutoHubScores() {
		latchedOutcome = AutoOutcome.TIE;
	} // End clearLatchedAutoHubScores

	/** Most recent auto outcome stored by {@link #latchAutoHubScoresFromFuelSim()}. */
	public static AutoOutcome getLatchedAutoOutcome() {
		return latchedOutcome;
	} // End getLatchedAutoOutcome

	/**
	 * Same as {@link #formatLineForSimulatedMatchBlue(int)} — blue hub score in the RP fuel field.
	 */
	public static String formatLineForSimulatedMatch(int autoTowerPoints) {
		return formatLineForSimulatedMatchBlue(autoTowerPoints);
	} // End formatLineForSimulatedMatch

	/** RP fuel field uses {@link FuelSim.Hub#BLUE_HUB} score. */
	public static String formatLineForSimulatedMatchBlue(int autoTowerPoints) {
		return formatLineForSimulatedMatchInternal(false, autoTowerPoints);
	} // End formatLineForSimulatedMatchBlue

	/** RP fuel field uses {@link FuelSim.Hub#RED_HUB} score. */
	public static String formatLineForSimulatedMatchRed(int autoTowerPoints) {
		return formatLineForSimulatedMatchInternal(true, autoTowerPoints);
	} // End formatLineForSimulatedMatchRed

	private static String formatLineForSimulatedMatchInternal(boolean redHubRpField, int autoTowerPoints) {
		HubShiftUtil.ShiftInfo shift = HubShiftUtil.getOfficialShiftInfo();
		int fuel = redHubRpField ? FuelSim.Hub.RED_HUB.getScore() : FuelSim.Hub.BLUE_HUB.getScore();
		return formatTeamSignLine(latchedOutcome, shift, fuel, autoTowerPoints, SimMatchTimeCache.getRemainingSec());
	} // End formatLineForSimulatedMatchInternal

	/**
	 * {@code current/threshold} fuel field. Shows {@value #kEnergizedFuelRpThreshold} until Energized,
	 * then {@value #kSuperchargedFuelRpThreshold} toward Supercharged.
	 */
	public static String fuelRpProgressString(int fuelScoredForDisplay) {
		int shown = Math.max(0, fuelScoredForDisplay);
		int denom = shown < kEnergizedFuelRpThreshold ? kEnergizedFuelRpThreshold : kSuperchargedFuelRpThreshold;
		return shown + "/" + denom;
	} // End fuelRpProgressString

	/**
	 * Formats one team-sign line from explicit inputs.
	 *
	 * @param autoOutcome latched auto outcome (controls R/B alternation)
	 * @param shiftInfo current shift segment and remaining time in that segment
	 * @param fuelScoredForDisplay hub fuel count shown in the RP field (blue or red per caller)
	 * @param autoTowerPoints auto tower points shown as the standalone integer field
	 * @param matchTimeSec remaining match seconds (negative/NaN renders as {@code --:--})
	 */
	public static String formatTeamSignLine(
			AutoOutcome autoOutcome,
			HubShiftUtil.ShiftInfo shiftInfo,
			int fuelScoredForDisplay,
			int autoTowerPoints,
			double matchTimeSec) {

		char shiftLetter = shiftLetterFor(shiftInfo.currentShift(), autoOutcome);
		int shiftSecondsShown = shiftSecondsToDisplay(shiftInfo);
		String fuelField = fuelRpProgressString(fuelScoredForDisplay);
		String matchClock = formatMatchClock(matchTimeSec);

		return String.format("%c%d %s %d %s", shiftLetter, shiftSecondsShown, fuelField, autoTowerPoints, matchClock);
	} // End formatTeamSignLine

	/**
	 * Shift letter per manual: {@code A} auto, {@code T} transition, {@code R}/{@code B} single
	 * active hub, {@code E} endgame, {@code -} disabled. R/B alternation comes from
	 * {@link AutoOutcome#scheduleRedFirst()}.
	 */
	public static char shiftLetterFor(HubShiftUtil.ShiftEnum shift, AutoOutcome autoOutcome) {
		return switch (shift) {
			case AUTO -> 'A';
			case TRANSITION -> 'T';
			case ENDGAME -> 'E';
			case SHIFT1, SHIFT2, SHIFT3, SHIFT4 -> fieldActiveRedHub(autoOutcome.scheduleRedFirst(), shift) ? 'R' : 'B';
			case DISABLED -> '-';
		};
	} // End shiftLetterFor

	/** True when the red alliance hub is the sole active scoring hub for this shift segment. */
	public static boolean fieldActiveRedHub(boolean redFirstInSchedule, HubShiftUtil.ShiftEnum shift) {
		boolean oddShift = shift == HubShiftUtil.ShiftEnum.SHIFT1 || shift == HubShiftUtil.ShiftEnum.SHIFT3;
		return redFirstInSchedule != oddShift;
	} // End fieldActiveRedHub

	private static int shiftSecondsToDisplay(HubShiftUtil.ShiftInfo shiftInfo) {
		if (shiftInfo.currentShift() == HubShiftUtil.ShiftEnum.DISABLED) {
			return 0;
		}
		return (int) Math.ceil(Math.max(0.0, shiftInfo.remainingTime() - 1.0e-6)); // TODO: Check with Drive Team about Driverstation Display (https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf Page 30)
	} // End shiftSecondsToDisplay

	private static String formatMatchClock(double matchTimeSec) {
		if (matchTimeSec < 0.0 || Double.isNaN(matchTimeSec)) {
			return "--:--";
		}
		int totalSeconds = (int) Math.ceil(Math.max(0.0, matchTimeSec) - 1.0e-6);
		int minutes = totalSeconds / 60;
		int seconds = totalSeconds % 60;
		return String.format("%d:%02d", minutes, seconds);
	} // End formatMatchClock
}
