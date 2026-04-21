package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.FieldConstants;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;

/**
 * SIM-only hub-light renderer for AdvantageScope models.
 *
 * <p>Lights are represented by teleporting models on/off the field:
 * <ul>
 *   <li>On: hub-center pose from {@link FieldConstants}</li>
 *   <li>Off: parked off-field</li>
 * </ul>
 *
 * <p>Model log keys are {@code Robot_Hub-Blue} and {@code Robot_Hub-Red}.
 */
public final class HubLightSimDisplay {

  private static final String kBlueHubModelKey = "FieldSimulation/HubLight-Blue";
  private static final String kRedHubModelKey =  "FieldSimulation/HubLight-Red";

  private static final Pose3d kBlueHubOnPose = new Pose3d(FieldConstants.BLUE_HUB_LIGHT_3D, new Rotation3d());
  private static final Pose3d kRedHubOnPose =  new Pose3d(FieldConstants.RED_HUB_LIGHT_3D, new Rotation3d());
  private static final Pose3d kOffFieldPose =  new Pose3d(-50.0, -50.0, -5.0, new Rotation3d());

  private static final double kTransitionFlashIntervalSec = 1.0;
  private static final double kWarningFlashIntervalSec = 0.5;
  private static final double kWarningWindowSec = 3.0;

  private HubLightSimDisplay() {}

  /** Updates both hub-light model poses based on current shift and auto outcome. */
  public static void update() {
    double matchRemainingSec = SimMatchTimeCache.getRemainingSec();
    if (matchRemainingSec <= 0.0) {
      Logger.recordOutput(kBlueHubModelKey, kOffFieldPose);
      Logger.recordOutput(kRedHubModelKey, kOffFieldPose);
      return;
    }

    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();
    TeamSignDisplayUtil.AutoOutcome autoOutcome = TeamSignDisplayUtil.getLatchedAutoOutcome();

    boolean blueOn = false;
    boolean redOn = false;
    switch (shiftInfo.currentShift()) {
      case AUTO:
      case ENDGAME:
        blueOn = true;
        redOn = true;
        break;

      case TRANSITION:
        blueOn = true;
        redOn = true;
        // Flash the hub that will be inactive in SHIFT1.
        boolean redActiveInShift1 = TeamSignDisplayUtil.fieldActiveRedHub(
            autoOutcome.scheduleRedFirst(), HubShiftUtil.ShiftEnum.SHIFT1);
        boolean flashOnTransition = shiftInfo.remainingTime() <= kWarningWindowSec
            ? warningFlashOn(shiftInfo.remainingTime())
            : flashOn(shiftInfo.elapsedTime(), kTransitionFlashIntervalSec);
        if (redActiveInShift1) {
          blueOn = flashOnTransition;
        } else {
          redOn = flashOnTransition;
        }
        break;

      case SHIFT1:
      case SHIFT2:
      case SHIFT3:
      case SHIFT4:
        boolean redActive = TeamSignDisplayUtil.fieldActiveRedHub(autoOutcome.scheduleRedFirst(), shiftInfo.currentShift());
        redOn = redActive;
        blueOn = !redActive;

        // Last 3 sec before a hub deactivates: flash currently active hub.
        if (willActiveHubDeactivateNextShift(shiftInfo.currentShift()) && shiftInfo.remainingTime() <= kWarningWindowSec) {
          boolean flashOnWarning = warningFlashOn(shiftInfo.remainingTime());
          if (redActive) {
            redOn = flashOnWarning;
          } else {
            blueOn = flashOnWarning;
          }
        }
        break;

      case DISABLED:
      default:
        blueOn = false;
        redOn = false;
        break;
    }

    Logger.recordOutput(kBlueHubModelKey, blueOn ? kBlueHubOnPose : kOffFieldPose);
    Logger.recordOutput(kRedHubModelKey, redOn ? kRedHubOnPose : kOffFieldPose);
  } // End update

  /** Returns true when the active hub in this shift will become inactive in the next shift. */
  private static boolean willActiveHubDeactivateNextShift(HubShiftUtil.ShiftEnum shift) {
    return shift == HubShiftUtil.ShiftEnum.SHIFT1
        || shift == HubShiftUtil.ShiftEnum.SHIFT2
        || shift == HubShiftUtil.ShiftEnum.SHIFT3;
  } // End willActiveHubDeactivateNextShift

  /** Square-wave flash state for a given interval where interval is the toggle period. */
  private static boolean flashOn(double elapsedSec, double intervalSec) {
    return ((int) Math.floor(elapsedSec / intervalSec)) % 2 == 0;
  } // End flashOn

  /**
   * Warning flash for the last 3s before deactivation.
   *
   * <p>Exact intended sequence: 3.0 off, 2.5 on, 2.0 off, 1.5 on, 1.0 off, 0.5 on, 0.0 off.
   */
  private static boolean warningFlashOn(double remainingSec) {
    int phaseIndex = (int) Math.floor((kWarningWindowSec - Math.max(0.0, remainingSec)) / kWarningFlashIntervalSec);
    return (phaseIndex % 2) == 1;
  } // End warningFlashOn
}
