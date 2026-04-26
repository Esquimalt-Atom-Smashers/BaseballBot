package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Prints {@code new TableEntry(distance, hoodExitDeg, commandedHoodDeg, rpm, tofSec)} for {@link
 * frc.robot.subsystems.shooter.ShooterLookup}. Hub 1.0–6.0 m, passing 3–12 m, step 0.5 m; hood
 * clamped to {@link HoodConstants}. Seeded rows use {@code commandedHoodDeg == hoodExitDeg}. Run
 * {@code ./gradlew printShooterLookupSeed}.
 */
public final class ShooterLookupTableGenerator {

  private ShooterLookupTableGenerator() {}

  private record Row(double dNom, double hoodDeg, double rpm, double tofSec) {}

  private record Solve(double hoodDeg, double rpm, double tofSec) {}

  public static void main(String[] args) {
    List<Row> hub = buildRows(FieldConstants.BLUE_FUNNEL_TOP_CENTER_3D, new Translation2d(-1.0, 0.0), 1.0, 6.0, 0.5);
    List<Row> passing =
        buildRows(FieldConstants.BLUE_PASSING_SPOT_CENTER, new Translation2d(1.0, 0.0), 3.0, 12.0, 0.5);
    System.out.println("  private static final TableEntry[] HUB_ENTRIES =");
    System.out.println("      new TableEntry[] {");
    System.out.println(
        "        // distance_m, hood_exit_deg, hood_commanded_deg, flywheel_rpm @ additive=0, time_of_flight_s");
    printRows(hub);
    System.out.println("      };");
    System.out.println();
    System.out.println("  private static final TableEntry[] PASSING_ENTRIES =");
    System.out.println("      new TableEntry[] {");
    System.out.println(
        "        // distance_m, hood_exit_deg, hood_commanded_deg, flywheel_rpm @ additive=0, time_of_flight_s");
    printRows(passing);
    System.out.println("      };");
  }

  private static void printRows(List<Row> rows) {
    for (Row r : rows) {
      double hood = r.hoodDeg();
      System.out.printf(
          "        new TableEntry(%.2f, %.1f, %.1f, %.0f, %.2f),%n",
          r.dNom(), hood, hood, r.rpm(), r.tofSec());
    }
  }

  private static List<Row> buildRows(
      Translation3d target3d, Translation2d dirRaw, double dMin, double dMax, double dStep) {
    double n = dirRaw.getNorm();
    Translation2d dir =
        n > 1e-9
            ? new Translation2d(dirRaw.getX() / n, dirRaw.getY() / n)
            : new Translation2d(1.0, 0.0);
    Translation2d targetXy = target3d.toTranslation2d();
    Translation2d tRel =
        new Translation2d(ShooterConstants.robotToTurret.getX(), ShooterConstants.robotToTurret.getY());
    List<Row> out = new ArrayList<>();
    for (double dNom = dMin; dNom <= dMax + 1e-9; dNom += dStep) {
      Translation2d turretXy = targetXy.plus(dir.times(dNom));
      Rotation2d heading = targetXy.minus(turretXy).getAngle();
      Translation2d robotTrans = turretXy.minus(tRel.rotateBy(heading));
      Pose2d robot = new Pose2d(robotTrans, heading);
      Solve solve = solveForPose(robot, target3d);
      if (solve != null) {
        out.add(new Row(dNom, solve.hoodDeg(), solve.rpm(), solve.tofSec()));
      }
    }
    out.sort(Comparator.comparingDouble(Row::dNom));
    return out;
  }

  /** Funnel solve, clamp hood to mechanism limits, fixed-hood refit if clamped; null if invalid. */
  private static Solve solveForPose(Pose2d robot, Translation3d target3d) {
    ShooterCalculator.ShotData funnel =
        ShooterCalculator.iterativeMovingShotFromFunnelClearance(
            robot, new ChassisSpeeds(), target3d, ShooterConstants.kLookaheadIterations);
    double hoodRad = funnel.getHoodAngle().in(Radians);
    double exitMps = funnel.getExitVelocity().in(MetersPerSecond);
    if (!Double.isFinite(exitMps) || !Double.isFinite(hoodRad) || exitMps <= 0.05) {
      return null;
    }
    double hoodUse = MathUtil.clamp(hoodRad, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);
    ShooterCalculator.ShotData shot;
    if (Math.abs(hoodUse - hoodRad) > 1e-8) {
      shot =
          ShooterCalculator.iterativeMovingShotWithFixedHoodAngle(
              robot,
              new ChassisSpeeds(),
              target3d,
              hoodUse,
              ShooterConstants.kLookaheadIterations);
    } else {
      shot = funnel;
    }
    exitMps = shot.getExitVelocity().in(MetersPerSecond);
    hoodRad = shot.getHoodAngle().in(Radians);
    if (!Double.isFinite(exitMps) || exitMps <= 0.05) {
      return null;
    }
    double dist =
        ShooterCalculator.getDistanceToTarget(robot, shot.getTarget()).in(Meters);
    if (!Double.isFinite(dist)) {
      return null;
    }
    double surfaceMps =
        exitMps
            / ShooterConstants.kFlywheelSurfaceDivider
            * ShooterConstants.kExitVelocityCompensationMultiplier;
    double radPerSec =
        ShooterCalculator.linearToAngularVelocity(
                MetersPerSecond.of(surfaceMps), Meters.of(FlywheelConstants.kFlywheelRadiusMeters))
            .in(RadiansPerSecond);
    double rpm = Units.radiansPerSecondToRotationsPerMinute(radPerSec);
    double tof =
        ShooterCalculator.calculateTimeOfFlight(
                MetersPerSecond.of(exitMps), Radians.of(hoodRad), Meters.of(dist))
            .in(Seconds);
    return new Solve(Units.radiansToDegrees(hoodRad), rpm, tof);
  }
}
