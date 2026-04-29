package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
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
import frc.robot.subsystems.shooter.ShooterLookup;
import frc.robot.subsystems.shooter.hood.HoodConstants;

/**
 * Prints {@code new TableEntry(...)} lines using distances from {@link ShooterLookup} hub/pass
 * tables. {@code hoodAngleDeg} (column 2) is the calculator’s ideal exit hood at that pose;
 * {@code commandedHoodAngleDeg} and {@code flywheelRpm} come from {@link ShooterLookup#lookup} at
 * the row distance. {@code timeOfFlightSec} matches {@link ShooterLookupTableGenerator}: {@link
 * ShooterCalculator#calculateTimeOfFlight} with exit speed, hood, and horizontal range from the same
 * ideal calculator shot (a flat model using only table distance + commanded hood + RPM diverges
 * from tuned rows when RPM or hood are hand-edited off that solve).
 *
 * <p>No args: all hub rows then all passing rows. One or two args: single {@code <distanceMeters>
 * [hub|passing]}.
 *
 * <p>Gradle: {@code ./gradlew printShooterLookupPoint} (full table). Optional single row: {@code
 * -PlookupDistance} / env {@code LOOKUP_DISTANCE} (quote decimals on PowerShell); see {@code
 * build.gradle}.
 */
public final class ShooterLookupPointGenerator {
  private ShooterLookupPointGenerator() {}

  public static void main(String[] args) {
    if (args.length == 0) {
      printTableFromLookupKeys();
      return;
    }
    double distanceMeters = Double.parseDouble(args[0]);
    boolean hubShot = args.length < 2 || !"passing".equalsIgnoreCase(args[1]);
    if (!printOneRow(hubShot, distanceMeters)) {
      System.exit(1);
    }
  }

  /** For each distance in {@link ShooterLookup} hub/pass arrays, print one table row (if solvable). */
  private static void printTableFromLookupKeys() {
    double[] hubD = ShooterLookup.hubTableDistancesMeters();
    double[] passD = ShooterLookup.passingTableDistancesMeters();
    if (hubD.length > 0) {
      System.out.println("        // HUB - regenerated from ShooterLookup.hubTableDistancesMeters()");
      for (double d : hubD) {
        printOneRow(true, d);
      }
    }
    if (passD.length > 0) {
      System.out.println("        // PASSING - regenerated from ShooterLookup.passingTableDistancesMeters()");
      for (double d : passD) {
        printOneRow(false, d);
      }
    }
    if (hubD.length == 0 && passD.length == 0) {
      System.err.println("ShooterLookup hub and passing tables are empty; nothing to print.");
    }
  }

  /** Prints one {@code new TableEntry(...)} line; returns false if the shot could not be solved. */
  private static boolean printOneRow(boolean hubShot, double distanceMeters) {
    Translation3d target3d =
        hubShot ? FieldConstants.BLUE_FUNNEL_TOP_CENTER_3D : FieldConstants.BLUE_PASSING_SPOT_CENTER;
    Translation2d dir = hubShot ? new Translation2d(-1.0, 0.0) : new Translation2d(1.0, 0.0);
    Pose2d robot = poseForDistance(target3d, dir, distanceMeters);

    IdealShot ideal = solveIdealShot(robot, target3d);
    if (ideal == null) {
      System.err.printf(
          "Could not compute a valid shot (%s) at distance %.3f m; skipped.%n",
          hubShot ? "hub" : "passing", distanceMeters);
      return false;
    }

    ShooterLookup.Result current = ShooterLookup.lookup(hubShot, distanceMeters);
    double commandedHoodDeg = Units.radiansToDegrees(current.hoodAngleRad());
    double rpm = current.flywheelRpmTable();

    double tofSec =
        ShooterCalculator.calculateTimeOfFlight(
                MetersPerSecond.of(ideal.exitMps()),
                Radians.of(ideal.hoodExitRad()),
                Meters.of(ideal.horizontalRangeM()))
            .in(Seconds);
    if (!Double.isFinite(tofSec) || tofSec <= 0.0) {
      System.err.printf(
          "Invalid TOF (distance=%.3f m, commandedHood=%.1f deg, rpm=%.0f); skipped.%n",
          distanceMeters, commandedHoodDeg, rpm);
      return false;
    }

    System.out.printf(
        "        new TableEntry(%.2f, %.1f, %.1f, %.0f, %.2f),%n",
        distanceMeters, ideal.hoodExitDeg(), commandedHoodDeg, rpm, tofSec);
    return true;
  }

  private static Pose2d poseForDistance(Translation3d target3d, Translation2d dirRaw, double distanceMeters) {
    double n = dirRaw.getNorm();
    Translation2d dir =
        n > 1e-9
            ? new Translation2d(dirRaw.getX() / n, dirRaw.getY() / n)
            : new Translation2d(1.0, 0.0);
    Translation2d targetXy = target3d.toTranslation2d();
    Translation2d turretXy = targetXy.plus(dir.times(distanceMeters));
    Rotation2d heading = targetXy.minus(turretXy).getAngle();
    Translation2d tRel =
        new Translation2d(ShooterConstants.robotToTurret.getX(), ShooterConstants.robotToTurret.getY());
    Translation2d robotTrans = turretXy.minus(tRel.rotateBy(heading));
    return new Pose2d(robotTrans, heading);
  }

  /**
   * Ideal shot after funnel solve, hood clamp, and optional fixed-hood refit — same basis as {@link
   * ShooterLookupTableGenerator} for hood, exit speed, range, and TOF.
   */
  private record IdealShot(double hoodExitRad, double exitMps, double horizontalRangeM) {
    double hoodExitDeg() {
      return Units.radiansToDegrees(hoodExitRad);
    }
  }

  /** {@code null} if funnel / refit shot is invalid. */
  private static IdealShot solveIdealShot(Pose2d robot, Translation3d target3d) {
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
    double horizontalRangeM =
        ShooterCalculator.getDistanceToTarget(robot, shot.getTarget()).in(Meters);
    if (!Double.isFinite(horizontalRangeM)) {
      return null;
    }
    return new IdealShot(hoodRad, exitMps, horizontalRangeM);
  }
}
