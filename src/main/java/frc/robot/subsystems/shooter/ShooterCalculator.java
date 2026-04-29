// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.kScaleLinearVelocityBaseInPerS;
import static frc.robot.subsystems.shooter.ShooterConstants.kScaleLinearVelocityMultiplier;
import static frc.robot.subsystems.shooter.ShooterConstants.kScaleLinearVelocityPower;
import static frc.robot.subsystems.shooter.ShooterConstants.robotToTurret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;

/**
 * Physics-based shooter calculator: funnel clearance parabola and iterative moving shot.
 * All units: meters, radians, m/s. Hood angle is launch elevation from horizontal: 0 = parallel to
 * the floor, π/2 = straight up.
 */
public class ShooterCalculator {

    /** Turret pivot projected on the field XY (m), same frame as {@link #calculateAzimuthAngle}. */
    private static Translation2d turretTranslationOnField(Pose2d robot) {
        return new Pose3d(robot).transformBy(robotToTurret).toPose2d().getTranslation();
    } // End turretTranslationOnField

    /**
     * Horizontal distance (m) from the turret pivot XY to {@code target} on the floor (hub XY).
     * Used as ballistic range ({@code x_dist}), time-of-flight input, and funnel scaling ratio so range
     * matches where the shot aims from.
     */
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(turretTranslationOnField(robot).getDistance(target.toTranslation2d()));
    } // End getDistanceToTarget

    /**
     * Same horizontal range as {@link #getDistanceToTarget} for the hub stored in {@code shot}
     * (predicted hub when iterating).
     */
    public static Distance getHorizontalRangeForShot(Pose2d robot, ShotData shot) {
        return getDistanceToTarget(robot, shot.getTarget());
    } // End getHorizontalRangeForShot

    // see https://www.desmos.com/geometry/l4edywkmha
    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = Units.metersToInches(target.getZ() - robotToTurret.getZ());
        double angle = Math.atan(
                ((vel * vel) + Math.sqrt(Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel)))
                        / (g * x_dist));
        return Radians.of(angle);
    }

    // calculates how long it will take for a projectile to travel a set distance given its initial velocity and angle
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = hoodAngle.in(Radians);
        double dist = distance.in(Meters);
        return Seconds.of(dist / (vel * Math.cos(angle)));
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    // calculates the angle of a Turret relative to the robot to hit a target
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
        return calculateAzimuthAngle(robot, target, 0.0);
    }

    /**
     * Same as above but picks shortest path using current Turret angle so the Turret does not spin
     * the long way; respects TurretConstants min/max.
     */
    public static Angle calculateAzimuthAngle(
            Pose2d robot, Translation3d target, double currentTurretAngleRad) {
        Translation2d direction = target.toTranslation2d().minus(turretTranslationOnField(robot));
        double angleRad =
                MathUtil.inputModulus(
                        direction.getAngle().minus(robot.getRotation()).getRadians(),
                        -Math.PI,
                        Math.PI);
        // Prefer ±2π so we stay within limits and take the shorter rotation
        if (currentTurretAngleRad > 0
                && angleRad + 2 * Math.PI <= TurretConstants.kMaxAngleRad) {
            angleRad += 2 * Math.PI;
        } else if (currentTurretAngleRad < 0
                && angleRad - 2 * Math.PI >= TurretConstants.kMinAngleRad) {
            angleRad -= 2 * Math.PI;
        }
        return Radians.of(angleRad);
    }

    /**
     * Time derivative of the robot-frame azimuth from turret pivot to a field-fixed XY point (rad/s),
     * matching the angle in {@link #calculateAzimuthAngle(Pose2d, Translation3d)}. Use as turret
     * velocity feedforward: when the pivot orbits the robot center during yaw, field bearing to the
     * hub changes, so {@code -omega} alone is incomplete.
     *
     * <p>Model: target fixed; pivot moves with the chassis. ψ = φ − θ with φ = atan2(w) in field and
     * θ robot heading, so dψ/dt = dφ/dt − ω.
     */
    public static double robotFramePivotToTargetAimRateRadPerSec(
            Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, Translation2d targetXY) {
        Translation2d rBody = new Translation2d(robotToTurret.getX(), robotToTurret.getY());
        double omega = robotRelativeSpeeds.omegaRadiansPerSecond;

        Translation2d pivotField = turretTranslationOnField(robotPose);
        Translation2d w = targetXY.minus(pivotField);
        double wNormSq = w.getNorm() * w.getNorm();
        if (wNormSq < 1e-8) {
            return -omega;
        }

        // Pivot velocity in robot frame (origin translation + ω×r), then rotate to field.
        Translation2d vPivotRobot =
                new Translation2d(
                        robotRelativeSpeeds.vxMetersPerSecond - omega * rBody.getY(),
                        robotRelativeSpeeds.vyMetersPerSecond + omega * rBody.getX());
        Translation2d vPivotField = vPivotRobot.rotateBy(robotPose.getRotation());

        double wx = w.getX();
        double wy = w.getY();
        double dwx = -vPivotField.getX();
        double dwy = -vPivotField.getY();
        double dPhiDt = (wx * dwy - wy * dwx) / wNormSq;
        return dPhiDt - omega;
    } // End robotFramePivotToTargetAimRateRadPerSec

    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    // Custom velocity ramp meant to minimize how fast the flywheels have to change speed
    public static LinearVelocity scaleLinearVelocity(Distance distanceToTarget) {
        double velocity =  kScaleLinearVelocityBaseInPerS + kScaleLinearVelocityMultiplier
                             * Math.pow(distanceToTarget.in(Inches), kScaleLinearVelocityPower);
        return InchesPerSecond.of(velocity);
    }

    // see https://www.desmos.com/calculator/ezjqolho6g
    public static ShotData calculateShotFromFunnelClearance(
            Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {
        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double y_dist =
                Units.metersToInches(predictedTarget.getZ() - robotToTurret.getZ());
        double g = 386;
        double r =
                Units.metersToInches(FieldConstants.FUNNEL_RADIUS_M)
                        * x_dist
                        / getDistanceToTarget(robot, actualTarget).in(Inches);
        double h =
                Units.metersToInches(FieldConstants.FUNNEL_HEIGHT_M + ShooterConstants.kDistanceAboveFunnelM);
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        // theta = atan(b) is launch slope dy/dx at the muzzle → elevation from horizontal.
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));
        if (Double.isNaN(v0) || Double.isNaN(theta)) {
            v0 = 0;
            theta = 0;
        }
        return new ShotData(InchesPerSecond.of(v0), Radians.of(theta), predictedTarget);
    }

    // use an iterative lookahead approach to determine shot parameters for a moving robot
    public static ShotData iterativeMovingShotFromFunnelClearance(
            Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(
                    shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        }

        return shot;
    }

    /**
     * Shot with a fixed hood elevation (e.g. when hood is disabled). Solves for velocity only so the
     * ball hits the target. No funnel clearance constraint.
     *
     * @param hoodElevationFromHorizontalRad launch angle from horizontal (radians)
     */
    public static ShotData calculateShotWithFixedHoodAngle(
            Pose2d robot, Translation3d predictedTarget, double hoodElevationFromHorizontalRad) {
        double xDist = getDistanceToTarget(robot, predictedTarget).in(Meters);
        double yDist = predictedTarget.getZ() - robotToTurret.getZ();
        double theta = hoodElevationFromHorizontalRad;
        double denom = 2 * Math.cos(theta) * Math.cos(theta) * (xDist * Math.tan(theta) - yDist);
        double v0Sq = (denom > 1e-6) ? (G_MPS2 * xDist * xDist) / denom : 0;
        double v0 = Math.sqrt(Math.max(0, v0Sq));
        if (Double.isNaN(v0)) v0 = 0;
        return new ShotData(MetersPerSecond.of(v0), Radians.of(hoodElevationFromHorizontalRad), predictedTarget);
    }

    /** Iterative moving shot with fixed hood angle (velocity-only solve each step). */
    public static ShotData iterativeMovingShotWithFixedHoodAngle(
            Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target,
            double hoodElevationFromHorizontalRad, int iterations) {
        ShotData shot = calculateShotWithFixedHoodAngle(robot, target, hoodElevationFromHorizontalRad);
        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotWithFixedHoodAngle(robot, predictedTarget, hoodElevationFromHorizontalRad);
            distance = getDistanceToTarget(robot, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        }
        return shot;
    }

    private static final double G_MPS2 = 9.81;

    /**
     * Lookup-table moving shot after TOF iteration: hood/RPM/TOF from {@link ShooterLookup} at the converged
     * horizontal range to the ghost target, and the aim point (predicted hub) for turret.
     */
    public record LookupTableMovingShot(
            double hoodExitAngleRad,
            double commandedHoodAngleRad,
            double flywheelRpmTable,
            double timeOfFlightSec,
            Translation3d aimTarget3d,
            double distanceKeyMeters) {}

    /**
     * Same idea as iterative funnel clearance, but hood/RPM/TOF come from {@link ShooterLookup} at each step.
     * Re-reads distance from turret pivot to the predicted target and interpolates table rows so moving shots stay
     * consistent with table TOF (tunable stationary).
     */
    public static LookupTableMovingShot iterativeMovingShotFromLookupTable(
            Pose2d robot,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations,
            boolean hubShot) {
        double distance = getDistanceToTarget(robot, target).in(Meters);
        ShooterLookup.Result lr = ShooterLookup.lookup(hubShot, distance);
        Time tof = Seconds.of(lr.timeOfFlightSec());
        Translation3d predicted = target;

        for (int i = 0; i < iterations; i++) {
            predicted = predictTargetPos(target, fieldSpeeds, tof);
            distance = getDistanceToTarget(robot, predicted).in(Meters);
            lr = ShooterLookup.lookup(hubShot, distance);
            tof = Seconds.of(lr.timeOfFlightSec());
        }
        return new LookupTableMovingShot(
                lr.hoodExitAngleRad(),
                lr.hoodAngleRad(),
                lr.flywheelRpmTable(),
                lr.timeOfFlightSec(),
                predicted,
                distance);
    }

    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
        public ShotData(LinearVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), target);
        }

        public LinearVelocity getExitVelocity() {
            return MetersPerSecond.of(this.exitVelocity);
        }

        public Angle getHoodAngle() {
            return Radians.of(this.hoodAngle);
        }

        public Translation3d getTarget() {
            return this.target;
        }
    }
}