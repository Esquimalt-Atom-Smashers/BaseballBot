package frc.robot.simulation;

/**
 * Logger and SmartDashboard path segments for an additional simulated robot instance. Values namespace output keys so
 * they do not collide with the default root.
 */
public final class SecondSimRobotOutputs {

  /** Absolute logger root path segment (leading slash, no trailing slash). */
  public static final String ROOT = "/SecondSimRobotOutputs";

  /**
   * {@link #ROOT} with a trailing slash, for concatenation with relative logger paths (same shape as an empty-prefix
   * primary concatenation, but under this root).
   */
  public static final String LOG_ROOT_PREFIX = ROOT + "/";

  private SecondSimRobotOutputs() {} // End SecondSimRobotOutputs Constructor

  /**
   * Returns a SmartDashboard table key prefix: {@code ROOT} without its leading slash, a slash, then {@code relativePath}.
   *
   * @param relativePath path segment after that root (no leading slash)
   */
  public static String smartDashboardPrefix(String relativePath) {
    return ROOT.substring(1) + "/" + relativePath;
  } // End smartDashboardPrefix
}
