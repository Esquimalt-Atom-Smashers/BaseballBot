// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.hood.HoodConstants;

/** Constants for the Shooter assembly (Turret position on robot, camera on Turret, etc.). */
public final class ShooterConstants {

  private ShooterConstants() {}

  /** An extra amount of distance the robot has to be past the alliance zone for auto selecting target to be enabled */
  public static final double kAutoSelectShootingTargetAllianceZoneTolerance = 1.5;

  /** When aiming at the hub, autoshoot requires at least this horizontal distance to hub center (m). */
  public static final double kMinHubAutoshootDistanceM = 1.0; // TODO: May need tuning

  /** Transform from robot center to Turret pivot. +X = forward, +Y = left, +Z = up (meters). */
  public static final Transform3d robotToTurret =
      new Transform3d(-0.07, -0.165, 0.45, new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)));

  /** Distance above funnel the trajectory must pass (20 in), meters. */
  public static final double kDistanceAboveFunnelM = Units.inchesToMeters(0.0); // TODO: Increase if balls shoot too low.

  /** Number of iterations for moving-shot lookahead. */
  public static final int kLookaheadIterations = 3;

  /** Phase delay (s) to predict pose forward so shot is for when ball leaves. */
  public static final double kPhaseDelaySec = 0.03;

  /**
   * After the flywheel leaves target velocity, {@link frc.robot.subsystems.shooter.Shooter#isReadyToShoot}
   * stays true for this long before reporting not ready. Timer resets when velocity is back on target.
   */
  public static final double kFlywheelOffTargetGraceSec = 0.1;

  /** Time before the hub is active that we treat the hub as active for the preshoot */
  public static final Time kActivePreshootTime = Seconds.of(2.0);
  
  /** Target aim offset (degrees). */
  public static final double kTargetAimOffsetDeg = 0.0;

  /** Multiplier on calculator exit velocity (e.g. air resistance). 1.0 = no change. */
  public static final double kExitVelocityCompensationMultiplier = 1.34; // TODO: make 1.19 for Lookup

  /**
   * Sim-only efficiency from flywheel surface speed to launched fuel speed. Models wheel slip/transfer
   * losses so sim can share the same exit-velocity compensation as real robot.
   */
  public static final double kSimFlywheelToFuelExitVelocityEfficiency =
      1.0 / kExitVelocityCompensationMultiplier;

  /**
   * Single flywheel at bottom: flywheel surface speed = exit velocity / this divider (e.g. 0.5
   * means spin flywheel 2x so ball gets desired speed).
   */
  public static final double kFlywheelSurfaceDivider = 0.5;

  /** Scale linear velocity: base in/s, multiplier, power for distance term (unused by current shot). */
  public static final double kScaleLinearVelocityBaseInPerS = 50.0;
  public static final double kScaleLinearVelocityMultiplier = 70.0;
  public static final double kScaleLinearVelocityPower = 0.3;

  /**
   * Hood elevation from horizontal (radians) when Hood is disabled / locked; velocity solve uses
   * this fixed launch angle. Matches {@link HoodConstants#kDisabledAngleRad}.
   */
  public static final double kFixedHoodAngleWhenDisabledRad = HoodConstants.kDisabledAngleRad;

  /** Transform from Turret pivot to camera (when camera is on Turret). */
  public static final Transform3d turretToCamera =
      new Transform3d(
          0.0, 0.0, 0.0, new Rotation3d(0.0, Math.toRadians(-22.2), Units.degreesToRadians(180.0)));
}
