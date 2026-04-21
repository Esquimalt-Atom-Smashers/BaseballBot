// Copyright (c) 2021-2026 Littleton Robotics
// Local copy with drivable ramps (hub + three boundary lines per ramp, front open).
// Based on org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt

package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import org.dyn4j.dynamics.Settings;
import org.ironmaple.simulation.SimulatedArena;

public class Arena2026Rebuilt extends SimulatedArena {

  protected boolean shouldClock = true;
  protected double clock = 0;
  protected boolean blueIsOnClock = Math.random() < 0.5;

  protected DoublePublisher phaseClockPublisher =
      genericInfoTable.getDoubleTopic("Time left in current phase").publish();
  protected BooleanPublisher redActivePublisher =
      redTable.getBooleanTopic("Red is active").publish();
  protected BooleanPublisher blueActivePublisher =
      blueTable.getBooleanTopic("Blue is active").publish();

  protected RebuiltHub blueHub;
  protected RebuiltHub redHub;
  protected RebuiltOutpost blueOutpost;
  protected RebuiltOutpost redOutpost;
  protected boolean isInEfficiencyMode = true;

  /**
   * Obstacles for the 2026 REBUILT field. When AddRampCollider is true, ramps are drivable: hub is
   * a solid rectangle; each ramp is three boundary lines (sides + back), leaving the front open so
   * the robot can drive onto the ramp surface.
   */
  public static final class RebuiltFieldObstaclesMap extends FieldMap {

    private static final double FIELD_X_MIN = 0.00000;
    private static final double FIELD_X_MAX = 16.54105;
    private static final double FIELD_Y_MIN = 0.00000;
    private static final double FIELD_Y_MAX = 8.06926;

    private static final double HUB_X_LEN = 1.19380;
    private static final double HUB_Y_LEN = 1.19380;
    private static final double HUB_X = 4.625594;
    private static final double HUB_Y = 4.03463;

    private static final double UPRIGHT_X_LEN = Inches.of(3.5).in(Meters);
    private static final double UPRIGHT_Y_LEN = Inches.of(1.5).in(Meters);
    private static final double UPRIGHT_OFFSET_FROM_END_WALL = 1.06204;
    private static final double UPRIGHT_OFFSET_FROM_SIDE_WALL = 3.31524;
    private static final double UPRIGHT_Y_SPACING = Inches.of(33.75).in(Meters);

    private static final double TRENCH_WALL_Y_LEN = Inches.of(12.0).in(Meters);
    private static final double TRENCH_WALL_X_LEN = Inches.of(47.0).in(Meters);
    private static final double TRENCH_WALL_OFFSET_FROM_END_WALL = 4.61769;
    private static final double TRENCH_WALL_OFFSET_FROM_SIDE_WALL = 1.43113;

    public RebuiltFieldObstaclesMap(boolean addRampCollider) {
      // Field border
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MIN, FIELD_Y_MAX));
      addBorderLine(
          new Translation2d(FIELD_X_MAX, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MIN));
      addBorderLine(
          new Translation2d(FIELD_X_MIN, FIELD_Y_MAX), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));

      // Blue tower uprights
      addRectangularObstacle(
          UPRIGHT_X_LEN,
          UPRIGHT_Y_LEN,
          new Pose2d(UPRIGHT_OFFSET_FROM_END_WALL, UPRIGHT_OFFSET_FROM_SIDE_WALL, new Rotation2d()));
      addRectangularObstacle(
          UPRIGHT_X_LEN,
          UPRIGHT_Y_LEN,
          new Pose2d(
              UPRIGHT_OFFSET_FROM_END_WALL,
              UPRIGHT_OFFSET_FROM_SIDE_WALL + UPRIGHT_Y_SPACING,
              new Rotation2d()));

      // Red tower uprights
      addRectangularObstacle(
          UPRIGHT_X_LEN,
          UPRIGHT_Y_LEN,
          new Pose2d(
              FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
              FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL,
              new Rotation2d()));
      addRectangularObstacle(
          UPRIGHT_X_LEN,
          UPRIGHT_Y_LEN,
          new Pose2d(
              FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
              FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL - UPRIGHT_Y_SPACING,
              new Rotation2d()));

      // Blue trench wall
      addRectangularObstacle(
          TRENCH_WALL_X_LEN,
          TRENCH_WALL_Y_LEN,
          new Pose2d(TRENCH_WALL_OFFSET_FROM_END_WALL, TRENCH_WALL_OFFSET_FROM_SIDE_WALL, new Rotation2d()));
      addRectangularObstacle(
          TRENCH_WALL_X_LEN,
          TRENCH_WALL_Y_LEN,
          new Pose2d(
              TRENCH_WALL_OFFSET_FROM_END_WALL,
              FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
              new Rotation2d()));

      // Red trench wall
      addRectangularObstacle(
          TRENCH_WALL_X_LEN,
          TRENCH_WALL_Y_LEN,
          new Pose2d(
              FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
              TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
              new Rotation2d()));
      addRectangularObstacle(
          TRENCH_WALL_X_LEN,
          TRENCH_WALL_Y_LEN,
          new Pose2d(
              FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
              FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
              new Rotation2d()));

      // Hub and ramps
      if (addRampCollider) {
        // Drivable ramps: hub solid, each ramp only has the back line (ramp–hub edge). No side or
        // front lines so the robot can drive onto the ramp without hitting a wall.
        double halfX = HUB_X_LEN / 2.0;
        double halfY = HUB_Y_LEN / 2.0;
        double blueLeftX = HUB_X - halfX;
        double blueRightX = HUB_X + halfX;
        double hubBottom = HUB_Y - halfY;
        double hubTop = HUB_Y + halfY;

        // Blue hub (solid)
        addRectangularObstacle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(HUB_X, HUB_Y, new Rotation2d()));

        // Blue -Y ramp: only back line (where ramp meets hub)
        addBorderLine(new Translation2d(blueLeftX, hubBottom), new Translation2d(blueRightX, hubBottom));
        // Blue +Y ramp: only back line
        addBorderLine(new Translation2d(blueLeftX, hubTop), new Translation2d(blueRightX, hubTop));

        // Red hub (solid)
        double redHubX = FIELD_X_MAX - HUB_X;
        double redLeftX = redHubX - halfX;
        double redRightX = redHubX + halfX;
        addRectangularObstacle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(redHubX, HUB_Y, new Rotation2d()));

        // Red -Y ramp: only back line
        addBorderLine(new Translation2d(redLeftX, hubBottom), new Translation2d(redRightX, hubBottom));
        // Red +Y ramp: only back line
        addBorderLine(new Translation2d(redLeftX, hubTop), new Translation2d(redRightX, hubTop));
      } else {
        addRectangularObstacle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(HUB_X, HUB_Y, new Rotation2d()));
        addRectangularObstacle(
            HUB_X_LEN, HUB_Y_LEN, new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()));
      }
    }
  }

  /** Creates arena with drivable ramps (true). */
  public Arena2026Rebuilt() {
    this(true);
  }

  /**
   * @param addRampCollider true = hub + drivable ramp boundaries; false = hub only, no ramp
   *     colliders.
   */
  public Arena2026Rebuilt(boolean addRampCollider) {
    super(new RebuiltFieldObstaclesMap(addRampCollider));

    Settings settings = physicsWorld.getSettings();
    settings.setMinimumAtRestTime(0.02);
    physicsWorld.setSettings(settings);

    blueHub = new RebuiltHub(this, true);
    super.addCustomSimulation(blueHub);
    redHub = new RebuiltHub(this, false);
    super.addCustomSimulation(redHub);
    blueOutpost = new RebuiltOutpost(this, true);
    super.addCustomSimulation(blueOutpost);
    redOutpost = new RebuiltOutpost(this, false);
    super.addCustomSimulation(redOutpost);
  }

  public static double randomInRange(double variance) {
    return (Math.random() - 0.5) * variance;
  }

  public void addPieceWithVariance(
      Translation2d piecePose,
      Rotation2d yaw,
      Distance height,
      LinearVelocity speed,
      Angle pitch,
      double xVariance,
      double yVariance,
      double yawVariance,
      double speedVariance,
      double pitchVariance) {
    addGamePieceProjectile(
        new RebuiltFuelOnFly(
            piecePose.plus(new Translation2d(randomInRange(xVariance), randomInRange(yVariance))),
            new Translation2d(),
            new ChassisSpeeds(),
            yaw.plus(Rotation2d.fromDegrees(randomInRange(yawVariance))),
            height,
            speed.plus(MetersPerSecond.of(randomInRange(speedVariance))),
            Degrees.of(pitch.in(Degrees) + randomInRange(pitchVariance))));
  }

  @Override
  public void placeGamePiecesOnField() {
    blueOutpost.reset();
    redOutpost.reset();

    // Field fuel is simulated only by {@link frc.robot.simulation.FuelSim} (spawn + physics +
    // intakes). Do not add duplicate {@link RebuiltFuelOnField} dyn4j bodies here — they doubled
    // CPU and piled on top of FuelSim.

    setupValueForMatchBreakdown("CurrentFuelInOutpost");
    setupValueForMatchBreakdown("TotalFuelInOutpost");
    setupValueForMatchBreakdown("TotalFuelInHub");
    setupValueForMatchBreakdown("WastedFuel");
  }

  @Override
  public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
    List<Pose3d> poses = super.getGamePiecesPosesByType(type);
    blueOutpost.draw(poses);
    redOutpost.draw(poses);
    return poses;
  }

  @Override
  public void simulationSubTick(int tickNum) {
    if (shouldClock && !DriverStation.isAutonomous() && DriverStation.isEnabled()) {
      clock -= getSimulationDt().in(Units.Seconds);
      if (clock <= 0) {
        clock = 25;
        blueIsOnClock = !blueIsOnClock;
      }
    } else {
      clock = 25;
    }
    phaseClockPublisher.set((clock));
    super.simulationSubTick(tickNum);
    blueActivePublisher.set(isActive(true));
    redActivePublisher.set(isActive(false));
  }

  public boolean isActive(boolean isBlue) {
    if (isBlue) {
      return blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
    } else {
      return !blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
    }
  }

  public void setShouldRunClock(boolean shouldRunClock) {
    shouldClock = shouldRunClock;
  }

  public void outpostDump(boolean isBlue) {
    (isBlue ? blueOutpost : redOutpost).dump();
  }

  public void outpostThrowForGoal(boolean isBlue) {
    (isBlue ? blueOutpost : redOutpost).throwForGoal();
  }

  public void outpostThrow(
      boolean isBlue, Rotation2d throwYaw, Angle throwPitch, LinearVelocity speed) {
    (isBlue ? blueOutpost : redOutpost).throwFuel(throwYaw, throwPitch, speed);
  }

  public void setEfficiencyMode(boolean efficiencyMode) {
    isInEfficiencyMode = efficiencyMode;
  }

  public boolean getEfficiencyMode() {
    return isInEfficiencyMode;
  }
}
