package frc.robot.util;

/** Telemetry helpers for consistent logger output formatting. */
public final class TelemetryUtil {

  private TelemetryUtil() {}

  /** Round a numeric telemetry value to two decimal places. */
  public static double roundToTwoDecimals(double value) {
    return Math.round(value * 100.0) / 100.0;
  } // End roundToTwoDecimals
}
