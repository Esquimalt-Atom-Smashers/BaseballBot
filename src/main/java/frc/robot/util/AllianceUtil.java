package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

/** Alliance-related helpers used across the robot (drive, shooter, etc.). */
public final class AllianceUtil {

  private AllianceUtil() {}

  /**
   * Returns true if the robot is on the red alliance, false otherwise (blue or unknown).
   *
   * @return true if on red alliance, false otherwise
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  } // End isRedAlliance

  /**
   * Returns true if the robot X position (m) is inside the current alliance's zone (depth
   * ALLIANCE_ZONE_M from that alliance's driver wall).
   */
  public static boolean isInAllianceZone(double robotXM) {
    return isInAllianceZone(robotXM, isRedAlliance());
  } // End isInAllianceZone

  /** Same as {@link #isInAllianceZone(double)} but with an explicit alliance (e.g. second sim robot). */
  public static boolean isInAllianceZone(double robotXM, boolean redAlliance) {
    return redAlliance
        ? robotXM >= FieldConstants.FIELD_LENGTH_M - FieldConstants.ALLIANCE_ZONE_M
        : robotXM <= FieldConstants.ALLIANCE_ZONE_M;
  } // End isInAllianceZone
}
