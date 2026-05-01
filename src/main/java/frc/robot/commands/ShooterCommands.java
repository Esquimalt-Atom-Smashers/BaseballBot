package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShooterCalculator.ShotData;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.simulation.SecondSimRobotOutputs;
import frc.robot.util.AllianceUtil;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Shooter-related command helpers (Hood, Flywheel, Turret aim via ShooterCalculator or optional ShooterLookup). */
public final class ShooterCommands {

  private ShooterCommands() {}

  private static final String kTargetAimOffsetDegKey = "Shooter/TargetAimOffsetDeg";
  private static final String kExitVelocityMultiplierAdditiveHubKey = "Shooter/ExitVelocityCompensationMultiplierAdditiveHub";
  private static final String kExitVelocityMultiplierAdditivePassingKey =  "Shooter/ExitVelocityCompensationMultiplierAdditivePassing";
  private static final String kUseLookupTableKey = "Shooter/UseLookupTable";
  private static final double kDefaultTargetAimOffsetDeg = 0.0;

  static {
    SmartDashboard.putNumber(kTargetAimOffsetDegKey, kDefaultTargetAimOffsetDeg);
    SmartDashboard.putNumber(kExitVelocityMultiplierAdditiveHubKey, ShooterConstants.kExitVelocityCompensationMultiplierAdditiveHub);
    SmartDashboard.putNumber(kExitVelocityMultiplierAdditivePassingKey, ShooterConstants.kExitVelocityCompensationMultiplierAdditivePassing);
    SmartDashboard.putBoolean(kUseLookupTableKey, false);
  }

  /** Per-{@link Drive} passing spot override; null/absent = aim at hub. */
  private static final Map<Drive, PassingSpot> passingSpotOverrideByDrive = new IdentityHashMap<>();

  /**
   * Per-{@link Drive} turret angle from last {@link #setShooterTarget} for that drive (moving shot). Static singleton
   * cache was wrong with two sim robots: whichever {@link frc.robot.subsystems.shooter.Shooter#periodic} ran last
   * overwrote the angle and broke the other robot’s turret aim.
   */
  private static final Map<Drive, Rotation2d> lastTurretAngleFromShotByDrive = new IdentityHashMap<>();

  /**
   * Optional per-{@link Drive} alliance for hub and passing-spot targets. Default when absent: {@link
   * AllianceUtil#isRedAlliance()} (DriverStation). The second sim robot registers its {@link Drive} with a supplier
   * driven by the SIM dashboard chooser (Blue / Red Alliance).
   */
  private static final IdentityHashMap<Drive, BooleanSupplier> targetRedAllianceByDrive =
      new IdentityHashMap<>();

  /** Call for the second sim drivetrain so {@link #getShooterTarget3d(Drive)} uses the same alliance as RobotContainer's second sim. */
  public static void registerTargetAllianceSupplier(Drive drive, BooleanSupplier isRedAlliance) {
    if (drive != null && isRedAlliance != null) {
      targetRedAllianceByDrive.put(drive, isRedAlliance);
    }
  } // End registerTargetAllianceSupplier

  private static boolean isRedAllianceForShooterTarget(Drive drive) {
    BooleanSupplier redAllianceSupplier = targetRedAllianceByDrive.get(drive);
    return redAllianceSupplier != null ? redAllianceSupplier.getAsBoolean() : AllianceUtil.isRedAlliance();
  } // End isRedAllianceForShooterTarget

  public enum PassingSpot {
    LEFT,
    CENTER,
    RIGHT
  }

  /** Set target to passing spot left (alliance-relative) for this drivetrain. */
  public static void setPassingSpotLeft(Drive drive) {
    if (drive != null) {
      passingSpotOverrideByDrive.put(drive, PassingSpot.LEFT);
    }
  }

  /** Set target to passing spot center (alliance-relative) for this drivetrain. */
  public static void setPassingSpotCenter(Drive drive) {
    if (drive != null) {
      passingSpotOverrideByDrive.put(drive, PassingSpot.CENTER);
    }
  }

  /** Set target to passing spot right (alliance-relative) for this drivetrain. */
  public static void setPassingSpotRight(Drive drive) {
    if (drive != null) {
      passingSpotOverrideByDrive.put(drive, PassingSpot.RIGHT);
    }
  }

  /** Clear override so Shooter returns to hub for this drivetrain. */
  public static void clearShooterTargetOverride(Drive drive) {
    if (drive != null) {
      passingSpotOverrideByDrive.remove(drive);
    }
  }

  /** True if current target is the hub (no passing-spot override) for this drivetrain. */
  public static boolean isShooterTargetHub(Drive drive) {
    return drive == null || passingSpotOverrideByDrive.get(drive) == null;
  }

  private static Translation3d getPassingSpot3d(PassingSpot spot, boolean redAlliance) {
    return switch (spot) {
      case LEFT -> redAlliance ? FieldConstants.RED_PASSING_SPOT_LEFT : FieldConstants.BLUE_PASSING_SPOT_LEFT;
      case CENTER ->
          redAlliance ? FieldConstants.RED_PASSING_SPOT_CENTER : FieldConstants.BLUE_PASSING_SPOT_CENTER;
      case RIGHT -> redAlliance ? FieldConstants.RED_PASSING_SPOT_RIGHT : FieldConstants.BLUE_PASSING_SPOT_RIGHT;
    };
  }

  /** Current Shooter target for this drivetrain: passing spot or alliance hub (funnel top). */
  public static Translation3d getShooterTarget3d(Drive drive) {
    boolean isRedAllianceForTarget = isRedAllianceForShooterTarget(drive);
    PassingSpot spot = passingSpotOverrideByDrive.get(drive);
    if (spot != null) return getPassingSpot3d(spot, isRedAllianceForTarget);
    return isRedAllianceForTarget
        ? FieldConstants.RED_FUNNEL_TOP_CENTER_3D
        : FieldConstants.BLUE_FUNNEL_TOP_CENTER_3D;
  } // End getShooterTarget3d

  /** Get current target for logging for this drivetrain. */
  public static String getShooterTargetName(Drive drive) {
    PassingSpot spot = passingSpotOverrideByDrive.get(drive);
    if (spot == null) return "Hub";
    return switch (spot) {
      case LEFT -> "Passing Left";
      case CENTER -> "Passing Center";
      case RIGHT -> "Passing Right";
    };
  }

  /**
   * Field-frame angle from Turret pivot to alliance hub (direction to aim in field).
   */
  public static Rotation2d getFieldAngleToHubFromPivot(Drive drive) {
    Pose2d pose = drive.getPose();
    return pose.getRotation().plus(
        Rotation2d.fromRadians(
            ShooterCalculator.calculateAzimuthAngle(pose, getShooterTarget3d(drive)).in(Radians)));
  }

  /**
   * Turret angle in robot frame (0 = robot forward) to aim at alliance hub from pivot.
   */
  public static Rotation2d getTurretAngleToHubFromPivot(Drive drive) {
    return Rotation2d.fromRadians(
        ShooterCalculator.calculateAzimuthAngle(drive.getPose(), getShooterTarget3d(drive)).in(Radians));
  }

  /**
   * Turret angle from last shot solution (aim at predicted target so ball + robot velocity hits hub).
   * Fallback to hub angle if shot not yet computed this cycle.
   */
  public static Rotation2d getTurretAngleFromShot(Drive drive) {
    Rotation2d cached = lastTurretAngleFromShotByDrive.get(drive);
    if (cached != null) {
      return cached;
    }
    return getTurretAngleToHubFromPivot(drive);
  } // End getTurretAngleFromShot

  /**
   * Sets Hood and Flywheel target from ShooterCalculator. When hoodEnabled is false, uses a fixed
   * Hood elevation and solves for velocity only. When true, uses funnel clearance + moving shot.
   * Applies phase delay, then iterative moving shot; clamps Hood to mechanism limits.
   * Turret aim uses predicted target and shortest-path azimuth.
   * When {@code enableCalculator} is false (e.g. manual override), Hood and Flywheel targets are
   * not updated so manual override controls are functional.
   * When {@code shootWhenReadyActive} is false and {@code hoodEnabled} is true, Hood target is set to
   * {@link HoodConstants#kDisabledAngleRad}; calculator hood angle is applied only while ShootWhenReady is active
   * ({@link frc.robot.subsystems.shooter.Shooter#isShootCommandActive()}).
   *
   * @param calculatorLogRoot AdvantageKit prefix; primary {@code ""}, second sim {@link
   *     SecondSimRobotOutputs#LOG_ROOT_PREFIX}. Calculator outputs log under {@code calculatorLogRoot + "Shooter/…"}.
   *     SmartDashboard calculator tuning uses the same {@code Shooter/…} keys for both robots. Logs {@code
   *     Shooter/DistanceTurretPivotToHubMeters}: horizontal distance from turret pivot to the aim point (hub or
 *     passing): calculator uses {@link ShooterCalculator#getHorizontalRangeForShot}; lookup uses {@link
 *     ShooterCalculator#iterativeMovingShotFromLookupTable} (TOF iteration to ghost target; logged distance key is the
 *     converged range). SmartDashboard {@code Shooter/UseLookupTable}
   *     selects lookup vs calculator when the hood path is active.
   */
  public static void setShooterTarget(
      Drive drive,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      boolean hoodEnabled,
      boolean enableCalculator,
      boolean shootWhenReadyActive,
      String calculatorLogRoot) {
    String logRoot = calculatorLogRoot != null ? calculatorLogRoot : "";

    Pose2d pose = drive.getPose();
    ChassisSpeeds fieldSpeeds = drive.getFieldRelativeChassisSpeeds();
    boolean isHubShot = isShooterTargetHub(drive);
    Translation3d target3d = getShooterTarget3d(drive);

    // Phase delay: predict pose forward so shot is for when ball actually leaves
    double phaseDelaySec = ShooterConstants.kPhaseDelaySec;
    Pose2d estimatedPose =
        new Pose2d(
            pose.getTranslation()
                .plus(
                    new Translation2d(
                        fieldSpeeds.vxMetersPerSecond * phaseDelaySec,
                        fieldSpeeds.vyMetersPerSecond * phaseDelaySec)),
            pose.getRotation()
                .plus(Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond * phaseDelaySec)));

    boolean useLookupTable = hoodEnabled && SmartDashboard.getBoolean(kUseLookupTableKey, false);
    Logger.recordOutput(logRoot + "Shooter/UseLookupTable", useLookupTable);

    ShotData shot = null;
    if (!hoodEnabled) {
      shot = ShooterCalculator.iterativeMovingShotWithFixedHoodAngle(
          estimatedPose,
          fieldSpeeds,
          target3d,
          ShooterConstants.kFixedHoodAngleWhenDisabledRad,
          ShooterConstants.kLookaheadIterations);
    } else if (!useLookupTable) {
      shot = ShooterCalculator.iterativeMovingShotFromFunnelClearance(
          estimatedPose, fieldSpeeds, target3d, ShooterConstants.kLookaheadIterations);
    }

    double exitVelMps;
    double hoodAngleRadFromSolve;
    Translation3d aimTarget3d;
    double distanceTurretPivotToHubM;
    double flywheelRadPerSec;

    double exitVelocityMultiplierAdditiveHub =
        SmartDashboard.getNumber(kExitVelocityMultiplierAdditiveHubKey, ShooterConstants.kExitVelocityCompensationMultiplierAdditiveHub);
    double exitVelocityMultiplierAdditivePassing =
        SmartDashboard.getNumber(kExitVelocityMultiplierAdditivePassingKey, ShooterConstants.kExitVelocityCompensationMultiplierAdditivePassing);
    double exitVelocityMultiplierAdditive =
        isHubShot ? exitVelocityMultiplierAdditiveHub : exitVelocityMultiplierAdditivePassing;

    if (useLookupTable) {
      ShooterCalculator.LookupTableMovingShot lookupShot =
          ShooterCalculator.iterativeMovingShotFromLookupTable(
              estimatedPose,
              fieldSpeeds,
              target3d,
              ShooterConstants.kLookaheadIterations,
              isHubShot);
      hoodAngleRadFromSolve = lookupShot.commandedHoodAngleRad();
      double rpmCmd =
          lookupShot.flywheelRpmTable()
              * (ShooterConstants.kExitVelocityCompensationMultiplier + exitVelocityMultiplierAdditive)
              / ShooterConstants.kExitVelocityCompensationMultiplier;
      flywheelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rpmCmd);
      aimTarget3d = lookupShot.aimTarget3d();
      distanceTurretPivotToHubM = lookupShot.distanceKeyMeters();
      double surfaceMps =
          ShooterCalculator.angularToLinearVelocity(
                  RadiansPerSecond.of(flywheelRadPerSec),
                  Meters.of(FlywheelConstants.kFlywheelRadiusMeters))
              .in(MetersPerSecond);
      exitVelMps =
          surfaceMps
              / (ShooterConstants.kExitVelocityCompensationMultiplier + exitVelocityMultiplierAdditive)
              * ShooterConstants.kFlywheelSurfaceDivider;
      Logger.recordOutput(logRoot + "Shooter/ShotSource", "Lookup");
      Logger.recordOutput(logRoot + "Shooter/LookupDistanceKeyMeters", lookupShot.distanceKeyMeters());
      Logger.recordOutput(logRoot + "Shooter/LookupCommandedHoodDeg", Units.radiansToDegrees(hoodAngleRadFromSolve));
      Logger.recordOutput(logRoot + "Shooter/LookupExitHoodDeg", Units.radiansToDegrees(lookupShot.hoodExitAngleRad()));
      Logger.recordOutput(logRoot + "Shooter/LookupFlywheelRpmTable", lookupShot.flywheelRpmTable());
      Logger.recordOutput(logRoot + "Shooter/LookupTimeOfFlightSec", lookupShot.timeOfFlightSec());
    } else {
      if (shot == null) {
        throw new IllegalStateException("Shooter shot solver did not run");
      }
      exitVelMps = shot.getExitVelocity().in(MetersPerSecond);
      hoodAngleRadFromSolve = shot.getHoodAngle().in(Radians);
      aimTarget3d = shot.getTarget();
      distanceTurretPivotToHubM =
          ShooterCalculator.getHorizontalRangeForShot(estimatedPose, shot).in(Meters);
      double flywheelSurfaceSpeedMps = exitVelMps / ShooterConstants.kFlywheelSurfaceDivider
              * (ShooterConstants.kExitVelocityCompensationMultiplier + exitVelocityMultiplierAdditive);
      flywheelRadPerSec =
          ShooterCalculator.linearToAngularVelocity(
                  MetersPerSecond.of(flywheelSurfaceSpeedMps),
                  Meters.of(FlywheelConstants.kFlywheelRadiusMeters))
              .in(RadiansPerSecond);
      Logger.recordOutput(logRoot + "Shooter/ShotSource", "Calculator");
    }

    Logger.recordOutput(logRoot + "Shooter/DistanceTurretPivotToHubMeters", distanceTurretPivotToHubM);
    Logger.recordOutput(logRoot + "Shooter/CalculatorHoodDeg", Units.radiansToDegrees(hoodAngleRadFromSolve));
    Logger.recordOutput(logRoot + "Shooter/CalculatorVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(flywheelRadPerSec));
    Logger.recordOutput(logRoot + "Shooter/ExitVelocityMps", exitVelMps);
    Logger.recordOutput(logRoot + "Shooter/ExitVelocityCompensationMultiplierAdditive", exitVelocityMultiplierAdditive);
    Logger.recordOutput(logRoot + "Shooter/ExitVelocityCompensationMultiplierAdditiveHub", exitVelocityMultiplierAdditiveHub);
    Logger.recordOutput(logRoot + "Shooter/ExitVelocityCompensationMultiplierAdditivePassing", exitVelocityMultiplierAdditivePassing);

    if (enableCalculator) {
      if (hoodEnabled) {
        if (shootWhenReadyActive) {
          double hoodAngleCommandRad = isHubShot
              ? Math.max(hoodAngleRadFromSolve, ShooterConstants.kHubMinHoodAngleRad) : hoodAngleRadFromSolve;
          hood.setTargetAngleRad(hoodAngleCommandRad);
          Logger.recordOutput(logRoot + "Shooter/HoodCommandDeg", Units.radiansToDegrees(hoodAngleCommandRad));
          Logger.recordOutput(logRoot + "Shooter/HubMinHoodAngleApplied", isHubShot);
        } else {
          hood.setTargetAngleRad(HoodConstants.kDisabledAngleRad);
        }
      }
      flywheel.setTargetVelocityRadPerSec(flywheelRadPerSec);
    }

    // We provide the turret "current angle" to the calculator in the turret's internal frame
    // so shortest-path selection respects TurretConstants travel limits.
    double targetAimOffsetDegAdditive = SmartDashboard.getNumber(kTargetAimOffsetDegKey, kDefaultTargetAimOffsetDeg);
    lastTurretAngleFromShotByDrive.put(
        drive,
        Rotation2d.fromRadians(ShooterCalculator.calculateAzimuthAngle(
            estimatedPose, aimTarget3d, turret.getPosition().getRadians())
            .in(Radians))
            .plus(Rotation2d.fromDegrees(ShooterConstants.kTargetAimOffsetDeg + targetAimOffsetDegAdditive)));
    Logger.recordOutput(logRoot + "Shooter/TargetAimOffsetDeg", ShooterConstants.kTargetAimOffsetDeg + targetAimOffsetDegAdditive);
  } // End setShooterTarget
}
