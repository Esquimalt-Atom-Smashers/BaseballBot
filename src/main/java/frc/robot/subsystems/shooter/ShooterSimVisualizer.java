// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Logs shooter aim trajectory and, when fuel physics is off, ballistic ghost balls (many may be active at once). */
public class ShooterSimVisualizer {

  private static final double GRAVITY_MPS2 = 9.81;
  private static final double TRAJECTORY_SAMPLE_DT_SEC = 0.04;
  private static final int TRAJECTORY_SAMPLE_COUNT = 50;

  private static final double GHOST_FUEL_RADIUS_M = 0.075;
  /** Ghost removal when age exceeds this; normal removal uses floor contact. */
  private static final double GHOST_MAX_AGE_SEC = 30.0;

  private Translation3d[] trajectory = new Translation3d[TRAJECTORY_SAMPLE_COUNT];
  private Supplier<Pose3d> poseSupplier;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final String logRoot;

  private final ArrayList<GhostProjectile> ghostFuels = new ArrayList<>();

  private static final class GhostProjectile {
    final double spawnTimeSec;
    final Translation3d launchPos;
    final Translation3d vel;

    GhostProjectile(double spawnTimeSec, Translation3d launchPos, Translation3d vel) {
      this.spawnTimeSec = spawnTimeSec;
      this.launchPos = launchPos;
      this.vel = vel;
    }
  }

  public ShooterSimVisualizer(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this(poseSupplier, fieldSpeedsSupplier, "");
  } // End ShooterSimVisualizer Constructor

  /** @param logRoot prefix for {@link Logger} keys under {@code ShooterVisualizer/…}; empty or a segment ending with {@code '/'} */
  public ShooterSimVisualizer(
      Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier, String logRoot) {
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    this.logRoot = logRoot != null ? logRoot : "";
  } // End ShooterSimVisualizer Constructor

  private Translation3d launchVel(LinearVelocity vel, Angle elevationAngle, Angle shotYawFieldFrame) {
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    double horizontalVel = Math.cos(elevationAngle.in(Radians)) * vel.in(MetersPerSecond);
    double verticalVel = Math.sin(elevationAngle.in(Radians)) * vel.in(MetersPerSecond);
    double yawRad = shotYawFieldFrame.in(Radians);
    double xVel = horizontalVel * Math.cos(yawRad);
    double yVel = horizontalVel * Math.sin(yawRad);

    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    return new Translation3d(xVel, yVel, verticalVel);
  } // End launchVel

  /**
   * @param vel launch speed magnitude
   * @param hoodAngleRad hood elevation from horizontal
   * @param turretYawRobotFrame turret yaw in robot frame; combined with robot heading for field-frame shot yaw
   */
  public void updateFuel(LinearVelocity vel, Angle hoodAngleRad, Angle turretYawRobotFrame) {
    Pose3d robotPose = poseSupplier.get();
    Angle elevationAngle = hoodAngleRad;
    Angle shotYawFieldFrame =
        Radians.of(
            robotPose.getRotation().toRotation2d().getRadians() + turretYawRobotFrame.in(Radians));
    Translation3d trajVel = launchVel(vel, elevationAngle, shotYawFieldFrame);
    Translation3d launchPos = robotPose.plus(ShooterConstants.robotToTurret).getTranslation();
    for (int i = 0; i < trajectory.length; i++) {
      double t = i * TRAJECTORY_SAMPLE_DT_SEC;
      double x = trajVel.getX() * t + launchPos.getX();
      double y = trajVel.getY() * t + launchPos.getY();
      double z = trajVel.getZ() * t - 0.5 * GRAVITY_MPS2 * t * t + launchPos.getZ();

      trajectory[i] = new Translation3d(x, y, z);
    }

    Logger.recordOutput(logRoot + "ShooterVisualizer/Trajectory", trajectory);
    updateGhostFuel();
  } // End updateFuel

  /**
   * Appends a ghost projectile with initial position and velocity; does not remove existing ghosts. Removal in
   * {@link #updateGhostFuel()} when the sphere center is at or below floor contact height or age exceeds {@link
   * #GHOST_MAX_AGE_SEC}.
   */
  public void startGhostFuel(LinearVelocity vel, Angle hoodElevationFromHorizontal, Angle turretYawRobotFrame) {
    Pose3d robot = poseSupplier.get();
    Angle elevationAngle = hoodElevationFromHorizontal;
    Angle shotYawFieldFrame =
        Radians.of(robot.getRotation().toRotation2d().getRadians() + turretYawRobotFrame.in(Radians));
    Translation3d initialVelocity = launchVel(vel, elevationAngle, shotYawFieldFrame);
    Translation3d launchPosition = robot.plus(ShooterConstants.robotToTurret).getTranslation();
    ghostFuels.add(new GhostProjectile(Timer.getFPGATimestamp(), launchPosition, initialVelocity));
  } // End startGhostFuel

  private void updateGhostFuel() {
    double now = Timer.getFPGATimestamp();
    ghostFuels.removeIf(
        projectile -> {
          double flightTimeSec = now - projectile.spawnTimeSec;
          if (flightTimeSec > GHOST_MAX_AGE_SEC) return true;
          double z =
              projectile.launchPos.getZ()
                  + projectile.vel.getZ() * flightTimeSec
                  - 0.5 * GRAVITY_MPS2 * flightTimeSec * flightTimeSec;
          return z <= GHOST_FUEL_RADIUS_M;
        });

    Pose3d[] poses = new Pose3d[ghostFuels.size()];
    for (int i = 0; i < ghostFuels.size(); i++) {
      GhostProjectile projectile = ghostFuels.get(i);
      double flightTimeSec = now - projectile.spawnTimeSec;
      double x = projectile.launchPos.getX() + projectile.vel.getX() * flightTimeSec;
      double y = projectile.launchPos.getY() + projectile.vel.getY() * flightTimeSec;
      double z =
          projectile.launchPos.getZ()
              + projectile.vel.getZ() * flightTimeSec
              - 0.5 * GRAVITY_MPS2 * flightTimeSec * flightTimeSec;
      poses[i] = new Pose3d(x, y, z, Rotation3d.kZero);
    }
    Logger.recordOutput(logRoot + "ShooterVisualizer/GhostFuel", poses);
  } // End updateGhostFuel

  /** Logs turret and hood poses in robot frame (turret at origin; hood offset and rotated by azimuth). */
  public void update3dPose(Angle azimuthAngle, Angle hoodAngle) {
    Pose3d turretPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, azimuthAngle.in(Radians)));
    Logger.recordOutput(logRoot + "ShooterVisualizer/TurretPose", turretPose);
    Pose3d hoodPose = new Pose3d(0.1, 0, 0, new Rotation3d(0, hoodAngle.in(Radians), 0));
    hoodPose = hoodPose.rotateAround(new Translation3d(), new Rotation3d(0, 0, azimuthAngle.in(Radians)));
    hoodPose =
        new Pose3d(
            hoodPose.getTranslation().plus(ShooterConstants.robotToTurret.getTranslation()),
            hoodPose.getRotation());
    Logger.recordOutput(logRoot + "ShooterVisualizer/HoodPose", hoodPose);
  } // End update3dPose
}
