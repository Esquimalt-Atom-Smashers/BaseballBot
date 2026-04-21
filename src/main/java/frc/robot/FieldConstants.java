package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Field layout constants including target positions for different alliances. */
public final class FieldConstants {

  /** Blue alliance hub center position in meters (2D, for pathfinding etc.). */
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6256194, 4.0346376);

  /** Red alliance hub center position in meters (2D, for pathfinding etc.). */
  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9154194, 4.0346376);

  /** Hub center height above floor. */
  private static final double HUB_CENTER_HEIGHT_M = 1.43256;

  /** Hub light height above floor. */
  public static final double HUB_LIGHT_HEIGHT_M = 1.275;

  /** Blue alliance hub center in 3D (x, y, z) meters for shooter ballistics. */
  public static final Translation3d BLUE_HUB_CENTER_3D =
      new Translation3d(BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY(), HUB_CENTER_HEIGHT_M);

  /** Red alliance hub center in 3D (x, y, z) meters for shooter ballistics. */
  public static final Translation3d RED_HUB_CENTER_3D =
      new Translation3d(RED_HUB_CENTER.getX(), RED_HUB_CENTER.getY(), HUB_CENTER_HEIGHT_M);

  /** Blue alliance hub light in 3D (x, y, z) meters for shooter ballistics. */
  public static final Translation3d BLUE_HUB_LIGHT_3D =
      new Translation3d(BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY(), HUB_LIGHT_HEIGHT_M);

  /** Red alliance hub light in 3D (x, y, z) meters for shooter ballistics. */
  public static final Translation3d RED_HUB_LIGHT_3D =
      new Translation3d(RED_HUB_CENTER.getX(), RED_HUB_CENTER.getY(), HUB_LIGHT_HEIGHT_M);

  /** Funnel radius. */
  public static final double FUNNEL_RADIUS_M = 0.6096;

  /** Funnel height. */
  public static final double FUNNEL_HEIGHT_M = 0.39624;

  /** Funnel top height above floor. */
  public static final double FUNNEL_TOP_HEIGHT_M = HUB_CENTER_HEIGHT_M + FUNNEL_HEIGHT_M;

  /** Blue alliance funnel top center in 3D (x, y, z) for shooter ballistics. */
  public static final Translation3d BLUE_FUNNEL_TOP_CENTER_3D =
      new Translation3d(BLUE_HUB_CENTER.getX(), BLUE_HUB_CENTER.getY(), FUNNEL_TOP_HEIGHT_M);

  /** Red alliance funnel top center in 3D (x, y, z) for shooter ballistics. */
  public static final Translation3d RED_FUNNEL_TOP_CENTER_3D =
      new Translation3d(RED_HUB_CENTER.getX(), RED_HUB_CENTER.getY(), FUNNEL_TOP_HEIGHT_M);

  /** Alliance zone depth. */
  public static final double ALLIANCE_ZONE_M = 3.977927;

  /** Field length (m) for mirroring red alliance positions. */
  public static final double FIELD_LENGTH_M = 16.54105;

  /** Field center Y (m). */
  public static final double FIELD_CENTER_Y_M = 4.021328;

  /** Field width (m). */
  public static final double FIELD_WIDTH_M = 2.0 * FIELD_CENTER_Y_M;

  // ---------- Trench and bump (for zone-based drive assist) ----------
  /** X position of center of trench/bump (m). ~181.56 in. */
  public static final double TRENCH_BUMP_X_M = 4.611624;
  /** Y width of trench (m). ~49.86 in. */
  public static final double TRENCH_WIDTH_M = 1.266444;
  /** X length of trench and bump (m). ~47 in. */
  public static final double TRENCH_BUMP_LENGTH_M = 1.1938;
  /** X width of trench bar (m). ~4 in. */
  public static final double TRENCH_BAR_WIDTH_M = 0.1016;
  /** Y width of block between bump and trench (m). ~12 in. */
  public static final double TRENCH_BLOCK_WIDTH_M = 0.3048;
  /** Y width of bump (m). ~73 in. */
  public static final double BUMP_WIDTH_M = 1.8542;
  /** Trench center Y (m). */
  public static final double TRENCH_CENTER_M = TRENCH_WIDTH_M / 2.0;

  /** Passing spot left (90 in from blue driver wall, 85 in left of field center). */
  public static final Translation3d BLUE_PASSING_SPOT_LEFT = new Translation3d(2.286, 6.180328, 0);

  /** Passing spot center (90 in from blue driver wall, field center y). */
  public static final Translation3d BLUE_PASSING_SPOT_CENTER = new Translation3d(2.286, 4.021328, 0);

  /** Passing spot right (90 in from blue driver wall, 85 in right of field center). */
  public static final Translation3d BLUE_PASSING_SPOT_RIGHT = new Translation3d(2.286, 1.862328, 0);

  /** Passing spot left (90 in from red driver wall; y mirrored so driver left is correct side). */
  public static final Translation3d RED_PASSING_SPOT_LEFT =
      new Translation3d(FIELD_LENGTH_M - 2.286, 2 * FIELD_CENTER_Y_M - 6.180328, 0);

  /** Passing spot center (90 in from red driver wall). */
  public static final Translation3d RED_PASSING_SPOT_CENTER =
      new Translation3d(FIELD_LENGTH_M - 2.286, FIELD_CENTER_Y_M, 0);

  /** Passing spot right (90 in from red driver wall; y mirrored). */
  public static final Translation3d RED_PASSING_SPOT_RIGHT =
      new Translation3d(FIELD_LENGTH_M - 2.286, 2 * FIELD_CENTER_Y_M - 1.862328, 0);

  private FieldConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
