package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import java.util.Arrays;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.hood.HoodConstants;

/**
 * Distance-based shooter table: hand-edited rows, linearly interpolated. Hub vs passing use
 * different tables (target height). Row distance is the lookup key (m); hub uses 1.0–7.5 m every
 * 0.5 m, passing 3.0–12.0 m every 0.5 m. {@code hoodAngleDeg} is the desired ball exit elevation
 * (deg); {@code hoodAngleDeg} is sent to the hood so realized exit matches {@code
 * hoodAngleDeg} (often equal at seed time). Both must stay within {@link HoodConstants#kMinAngleRad}
 * and {@link HoodConstants#kMaxAngleRad} (50°–80°).
 *
 * <p>Regenerate: {@code .\gradlew.bat printShooterLookupSeed}.
 */
public final class ShooterLookup {

  private ShooterLookup() {}

  /**
   * One row: range key (m), exit hood (deg), hood (deg), flywheel RPM at additive=0, TOF
   * (s). RPM matches {@code Shooter/CalculatorVelocityRpm} when {@code
   * Shooter/ExitVelocityCompensationMultiplierAdditive} is 0; runtime scales by {@code (kExit +
   * additive) / kExit}.
   */
  public record TableEntry(
      double distanceMeters,
      double idealExitDeg,
      double hoodAngleDeg,
      double flywheelRpm,
      double timeOfFlightSec) {}

  /**
   * Interpolated exit hood (rad, for logging), hood (rad, for {@link
   * frc.robot.subsystems.shooter.hood.Hood} setpoint), table RPM, TOF (s).
   */
  public record Result(
      double hoodExitAngleRad,
      double hoodAngleRad,
      double flywheelRpmTable,
      double timeOfFlightSec) {}

  private static final TableEntry[] HUB_ENTRIES =
      new TableEntry[] {
        // .85m constant - Add distance from Hub Wall to this value for distance value.
        // Need to tune TOF

        // distance_m, hood_exit_deg, hood_deg, flywheel_rpm @ additive=0, time_of_flight_s
        new TableEntry(0.90, 80.0, 80.0, 3300, 0.87),
        new TableEntry(1.15, 79.0, 80.0, 3000, 0.96),  // Soft balls launch at steeper angle (works from 2700 to 3000 rpm), hard balls at shallower angle (misses-hits hub).
        new TableEntry(1.47, 74.7, 80.0, 3000, 0.90),
        new TableEntry(1.85, 70.5, 68.0, 3000, 0.89),
        new TableEntry(2.25, 66.9, 65.0, 3000, 0.89),
        new TableEntry(2.65, 63.9, 63.0, 3200, 0.91),
      };

  private static final TableEntry[] PASSING_ENTRIES =
      new TableEntry[] {
        // distance_m, hood_exit_deg, hood_deg, flywheel_rpm @ additive=0, time_of_flight_s
        new TableEntry( 3.00, 50.0, 50.0, 2305, 0.91),
        new TableEntry( 3.50, 50.0, 50.0, 2510, 0.97),
        new TableEntry( 4.00, 50.0, 50.0, 2700, 1.03),
        new TableEntry( 4.50, 50.0, 50.0, 2877, 1.09),
        new TableEntry( 5.00, 50.0, 50.0, 3045, 1.14),
        new TableEntry( 5.50, 50.0, 50.0, 3203, 1.20),
        new TableEntry( 6.00, 50.0, 50.0, 3355, 1.24),
        new TableEntry( 6.50, 50.0, 50.0, 3500, 1.29),
        new TableEntry( 7.00, 50.0, 50.0, 3639, 1.34),
        new TableEntry( 7.50, 50.0, 50.0, 3773, 1.38),
        new TableEntry( 8.00, 50.0, 50.0, 3903, 1.43),
        new TableEntry( 8.50, 50.0, 50.0, 4028, 1.47),
        new TableEntry( 9.00, 50.0, 50.0, 4150, 1.51),
        new TableEntry( 9.50, 50.0, 50.0, 4268, 1.55),
        new TableEntry(10.00, 50.0, 50.0, 4383, 1.59),
        new TableEntry(10.50, 50.0, 50.0, 4495, 1.63),
        new TableEntry(11.00, 50.0, 50.0, 4605, 1.66),
        new TableEntry(11.50, 50.0, 50.0, 4712, 1.70),
        new TableEntry(12.00, 50.0, 50.0, 4816, 1.73),
      };

  /**
   * Interpolates exit vs hood (rad), flywheel RPM (table; scale exit-velocity additive in
   * caller), and time of flight (s) for the given horizontal range key.
   */
  public static Result lookup(boolean hubShot, double distanceMeters) {
    TableEntry[] table = hubShot ? HUB_ENTRIES : PASSING_ENTRIES;
    return interpolate(table, distanceMeters);
  }

  /**
   * Hub table row distances (m), in order. Used by tooling (e.g. {@link frc.robot.ShooterLookupPointGenerator})
   * to regenerate rows without duplicating keys.
   */
  public static double[] hubTableDistancesMeters() {
    return Arrays.stream(HUB_ENTRIES).mapToDouble(TableEntry::distanceMeters).toArray();
  }

  /**
   * Passing table row distances (m), in order. Used by tooling to regenerate rows without duplicating
   * keys.
   */
  public static double[] passingTableDistancesMeters() {
    return Arrays.stream(PASSING_ENTRIES).mapToDouble(TableEntry::distanceMeters).toArray();
  }

  private static Result interpolate(TableEntry[] table, double distanceMeters) {
    double minDeg = Math.toDegrees(HoodConstants.kMinAngleRad);
    double maxDeg = Math.toDegrees(HoodConstants.kMaxAngleRad);
    if (table.length == 0) {
      return new Result(0.0, 0.0, 0.0, 0.0);
    }
    if (distanceMeters <= table[0].distanceMeters()) {
      TableEntry e = table[0];
      double exitDeg = MathUtil.clamp(e.idealExitDeg(), minDeg, maxDeg);
      double cmdDeg = MathUtil.clamp(e.hoodAngleDeg(), minDeg, maxDeg);
      return new Result(
          Units.degreesToRadians(exitDeg),
          Units.degreesToRadians(cmdDeg),
          e.flywheelRpm(),
          e.timeOfFlightSec());
    }
    TableEntry last = table[table.length - 1];
    if (distanceMeters >= last.distanceMeters()) {
      double exitDeg = MathUtil.clamp(last.idealExitDeg(), minDeg, maxDeg);
      double cmdDeg = MathUtil.clamp(last.hoodAngleDeg(), minDeg, maxDeg);
      return new Result(
          Units.degreesToRadians(exitDeg),
          Units.degreesToRadians(cmdDeg),
          last.flywheelRpm(),
          last.timeOfFlightSec());
    }
    int lower = 0;
    while (lower + 1 < table.length && table[lower + 1].distanceMeters() < distanceMeters) {
      lower++;
    }
    TableEntry lo = table[lower];
    TableEntry hi = table[lower + 1];
    double t =
        MathUtil.inverseInterpolate(lo.distanceMeters(), hi.distanceMeters(), distanceMeters);
    double exitDeg = MathUtil.interpolate(lo.idealExitDeg(), hi.idealExitDeg(), t);
    exitDeg = MathUtil.clamp(exitDeg, minDeg, maxDeg);
    double cmdDeg = MathUtil.interpolate(lo.hoodAngleDeg(), hi.hoodAngleDeg(), t);
    cmdDeg = MathUtil.clamp(cmdDeg, minDeg, maxDeg);
    double rpm = MathUtil.interpolate(lo.flywheelRpm(), hi.flywheelRpm(), t);
    double tof = MathUtil.interpolate(lo.timeOfFlightSec(), hi.timeOfFlightSec(), t);
    return new Result(Units.degreesToRadians(exitDeg), Units.degreesToRadians(cmdDeg), rpm, tof);
  }
}
