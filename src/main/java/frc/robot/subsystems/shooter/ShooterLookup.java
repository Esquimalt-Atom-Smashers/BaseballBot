package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.Comparator;

/**
 * Lookup table for Shooter: distance to hub → Hood angle and Flywheel velocity.
 * One row per distance; linear interpolation between rows. Extrapolation clamps to first/last row.
 */
public final class ShooterLookup {

  private ShooterLookup() {}

  /** One table entry: distance (m), hood angle (rad), flywheel velocity (rad/s). */
  public record Entry(double distanceMeters, double angleRad, double velocityRadsPerSec) {
    public static Entry of(double distanceMeters, double angleDegrees, double velocityRpm) {
      return new Entry(
          distanceMeters,
          Units.degreesToRadians(angleDegrees),
          Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm));
    }
  }

  /** Result of a lookup: hood angle (rad) and flywheel velocity (rad/s). */
  public record Result(double angleRad, double velocityRadsPerSec) {}

  /** Distance (m), Hood Angle (deg), Flywheel Velocity (rpm) */
  private static final Entry[] TABLE = {
    Entry.of(2.0, 25.0, 3000.0),
    Entry.of(4.0, 35.0, 3500.0),
    Entry.of(6.0, 45.0, 4000.0),
  };

  static {
    Arrays.sort(TABLE, Comparator.comparingDouble(Entry::distanceMeters));
  }

  /**
   * Get Hood angle and Flywheel velocity for a given distance to hub (meters).
   * Linear interpolation between table rows; outside range clamps to first/last row.
   */
  public static Result get(double distanceMeters) {
    // Empty table: return safe default
    if (TABLE.length == 0) {
      return new Result(0.0, 0.0);
    }

    // Below table range: use first row (e.g. distance 1.5 m when first row is 2 m)
    if (distanceMeters <= TABLE[0].distanceMeters()) {
      return new Result(TABLE[0].angleRad(), TABLE[0].velocityRadsPerSec());
    }

    // Above table range: use last row (e.g. distance 7 m when last row is 6 m)
    Entry lastEntry = TABLE[TABLE.length - 1];
    if (distanceMeters >= lastEntry.distanceMeters()) {
      return new Result(lastEntry.angleRad(), lastEntry.velocityRadsPerSec());
    }

    // Find the two table rows that bracket the requested distance
    int lowerIndex = 0;
    while (lowerIndex + 1 < TABLE.length
        && TABLE[lowerIndex + 1].distanceMeters() < distanceMeters) {
      lowerIndex++;
    }
    Entry entryAtLowerDistance = TABLE[lowerIndex];
    Entry entryAtUpperDistance = TABLE[lowerIndex + 1];

    // Linear interpolation: blend factor 0 = at lower row, 1 = at upper row
    double blendFactor =
        MathUtil.inverseInterpolate(
            entryAtLowerDistance.distanceMeters(),
            entryAtUpperDistance.distanceMeters(),
            distanceMeters);
    double interpolatedAngleRad =
        MathUtil.interpolate(
            entryAtLowerDistance.angleRad(), entryAtUpperDistance.angleRad(), blendFactor);
    double interpolatedVelocityRadsPerSec =
        MathUtil.interpolate(
            entryAtLowerDistance.velocityRadsPerSec(),
            entryAtUpperDistance.velocityRadsPerSec(),
            blendFactor);

    return new Result(interpolatedAngleRad, interpolatedVelocityRadsPerSec);
  } // End get
}
