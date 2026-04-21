// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Modifications Copyright (c) 2026 Esquimalt Robotics

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.AllianceUtil;
import org.littletonrobotics.junction.Logger;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriveCommands {
  // ============================================================================
  // Constants
  // ============================================================================

  // PID constants for angle control
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  // Drive control constants
  private static final double XBOX_JOYSTICK_DEADBAND = 0.15; // Deadband threshold for joystick inputs
  private static final double MAX_CONTROL_SPEED = 1.6; // Max speed the driver can go in x or y in m/s
  private static final double MAX_ANGULAR_RATE = 0.75 * 2 * Math.PI; // 3/4 rotation per second in rad/s
  private static final double TURBO_MULTIPLE = 2.0; // Minimum fraction when turbo is not applied

  /** Robot-centric backup speed for {@link #timedDriveBackRobotCentric(Drive)} (m/s). */
  private static final double AUTO_DRIVE_BACK_SPEED_MPS = 0.25;

  /** Duration for {@link #timedDriveBackRobotCentric(Drive)} (s). */
  private static final double AUTO_DRIVE_BACK_TIMEOUT_SEC = 0.5;

  // Characterization constants
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  // ============================================================================
  // Public Getters
  // ============================================================================

  /** Returns the PID proportional gain for angle control. */
  public static double getAngleKp() {
    return ANGLE_KP;
  } // End getAngleKp

  /** Returns the PID derivative gain for angle control. */
  public static double getAngleKd() {
    return ANGLE_KD;
  } // End getAngleKd

  /** Returns the maximum angular velocity for angle control (rad/s). */
  public static double getAngleMaxVelocity() {
    return ANGLE_MAX_VELOCITY;
  } // End getAngleMaxVelocity

  /** Returns the maximum angular acceleration for angle control (rad/s²). */
  public static double getAngleMaxAcceleration() {
    return ANGLE_MAX_ACCELERATION;
  } // End getAngleMaxAcceleration

  /** Returns the maximum control speed for driver input (m/s). */
  public static double getMaxControlSpeed() {
    return MAX_CONTROL_SPEED;
  } // End getMaxControlSpeed

  /** Returns the maximum angular rate for driver input (rad/s). */
  public static double getMaxAngularRate() {
    return MAX_ANGULAR_RATE;
  } // End getMaxAngularRate

  /**
   * Returns the angular rate (rad/s) to rotate the robot toward the alliance hub.
   *
   * @param drive Drive subsystem (for pose and heading)
   * @param faceTargetController ProfiledPIDController for rotation
   * @return Angular velocity in rad/s (positive = CCW)
   */
  public static double computeOmegaToFaceHub(Drive drive, ProfiledPIDController faceTargetController) {
    Rotation2d angleFromPivotToHub = ShooterCommands.getFieldAngleToHubFromPivot(drive);
    Rotation2d targetAngle = angleFromPivotToHub.plus(TurretConstants.kDefaultAimDirectionRobotFrame);
    return faceTargetController.calculate(drive.getRotation().getRadians(), targetAngle.getRadians());
  } // End computeOmegaToFaceHub

  /**
   * Drives straight backward in the robot frame at constant speed, then stops. Intended for
   * PathPlanner named commands.
   */
  public static Command timedDriveBackRobotCentric(Drive drive) {
    return Commands.runEnd(
            () -> drive.runVelocity(new ChassisSpeeds(-AUTO_DRIVE_BACK_SPEED_MPS, 0.0, 0.0)),
            drive::stop,
            drive)
        .withTimeout(AUTO_DRIVE_BACK_TIMEOUT_SEC);
  } // End timedDriveBackRobotCentric

  // ============================================================================
  // Helper Methods
  // ============================================================================

  // ----------------------------------------------------------------------------
  // Input Processing Helpers
  // ----------------------------------------------------------------------------

  /**
   * Applies turbo scaling to an input value. The input should already have deadband and squaring
   * applied if needed.
   *
   * <p>Behavior:
   * <ul>
   *   <li>Interpolates `turboAxis` (expected in [0, 1]) linearly between a minimum turbo value
   *       (1 / TURBO_MULTIPLE) and 1.0, then scales the result.
   * </ul>
   *
   * @param inputValue Input value that has already been processed (deadbanded, squared, etc.).
   *     Positive/negative direction is preserved.
   * @param turboAxis Turbo level in [0.0, 1.0] (0 = minimum speed limiter; 1 = full speed).
   * @param maxRange Maximum magnitude of the output (units chosen by caller; e.g. meters/sec or
   *     radians/sec). Should be >= 0.
   * @return The scaled output value with turbo applied
   */
  public static double scaleAxisWithTurbo(double inputValue, double turboAxis, double maxRange) {
    // Linear interpolation from minTurbo (when turboAxis == 0) to 1.0 (when turboAxis == 1).
    double minTurbo = 1.0 / TURBO_MULTIPLE; // Minimum fraction of maxRange when turbo is not applied
    double turbo = turboAxis * (1.0 - minTurbo) + minTurbo;

    // Scale the (signed) value by the physical range and the turbo multiplier
    return inputValue * maxRange * turbo;
  } // End scaleAxisWithTurbo

  /**
   * Converts joystick inputs to a linear velocity vector. (Squares input for joystick input shaping.)
   * 
   * @param x axis joystick input
   * @param y axis joystick input
   * @return linear velocity vector
   */
  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), XBOX_JOYSTICK_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  } // End getLinearVelocityFromJoysticks


  // ----------------------------------------------------------------------------
  // Alliance/Field-Relative Helpers
  // ----------------------------------------------------------------------------

  /**
   * Converts robot-relative speeds to field-relative speeds, accounting for alliance color.
   *
   * @param robotRelativeSpeeds Speeds relative to the robot's current orientation
   * @param drive The drive subsystem to get current rotation from
   * @return Speeds relative to the field, flipped if on red alliance
   */
  private static ChassisSpeeds convertToFieldRelative(ChassisSpeeds robotRelativeSpeeds, Drive drive) {
    // For mirrored field layout with "Always blue origin" approach:
    // - Origin is always on blue side (AprilTags use this)
    // - When on red alliance, rotate the reference frame by 180° instead of inverting speeds
    // - Robot orientation already accounts for which side it's on (from vision or simulation)
    
    boolean isFlipped = AllianceUtil.isRedAlliance();
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        robotRelativeSpeeds,
        isFlipped
            ? drive.getRotation().plus(new Rotation2d(Math.PI))
            : drive.getRotation());
  }

  /**
   * Converts robot-relative speeds to field-relative speeds and executes the drive command.
   * This helper method follows the DRY principle by centralizing the common pattern used across
   * all drive commands.
   *
   * @param drive The drive subsystem
   * @param vx Robot-relative velocity in X direction (m/s)
   * @param vy Robot-relative velocity in Y direction (m/s)
   * @param omega Robot-relative angular velocity (rad/s)
   */
  private static void driveFieldRelative(Drive drive, double vx, double vy, double omega) {
    ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(vx, vy, omega);
    ChassisSpeeds fieldRelativeSpeeds = convertToFieldRelative(robotRelativeSpeeds, drive);
    drive.runVelocity(fieldRelativeSpeeds);
  } // End driveFieldRelative


  // ----------------------------------------------------------------------------
  // Target Calculation Helpers
  // ----------------------------------------------------------------------------

  /**
   * Calculates the target angle to face the alliance's hub center.
   *
   * @param drive The drive subsystem to get current robot pose from
   * @return The desired rotation to face the target hub
   */
  private static Rotation2d calculateTargetHubAngle(Drive drive) {
    // Get robot's current position
    Pose2d robotPose = drive.getPose();
    Translation2d robotPosition = robotPose.getTranslation();

    // Determine target based on alliance
    Translation2d targetPosition =
        AllianceUtil.isRedAlliance() ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;

    // Calculate angle from robot to target
    Translation2d delta = targetPosition.minus(robotPosition);
    return new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
  } // End calculateTargetHubAngle

  /**
   * Calculates the Turret hub angle (robot frame) to aim at the alliance hub using robot pose.
   *
   * @param drive The drive subsystem to get current robot pose from
   * @return Desired Turret angle in robot frame (0 = robot forward)
   */
  public static Rotation2d getTurretAngleToHub(Drive drive) {
    Rotation2d angleToHubField = calculateTargetHubAngle(drive);
    return angleToHubField.minus(drive.getPose().getRotation());
  }

  // ============================================================================
  // Public Drive Commands
  // ============================================================================

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), XBOX_JOYSTICK_DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          driveFieldRelative(
              drive,
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
        },
        drive);
  } // End joystickDrive

  /**
   * Field-relative drive command with turbo control and optional face-target mode.
   *
   * <p>This command supports:
   * <ul>
   *   <li>Turbo control via trigger input (scales speed between min and max)
   *   <li>Face-target mode (automatically rotates toward hub when enabled)
   *   <li>Standard joystick control when face-target is disabled
   * </ul>
   *
   * @param drive The drive subsystem
   * @param xSupplier Supplier for X-axis joystick input (left/right)
   * @param ySupplier Supplier for Y-axis joystick input (forward/backward)
   * @param omegaSupplier Supplier for omega (rotation) joystick input (only used when face-target is disabled)
   * @param turboSupplier Supplier for turbo level [0.0, 1.0] (typically right trigger)
   * @param faceTargetEnabledSupplier Supplier indicating if face-target mode is enabled
   * @param isRobotCentricSupplier Supplier indicating if drive is robot-centric (true) or field-centric (false)
   * @param faceTargetController ProfiledPIDController for face-target rotation control
   * @param usePhysicalMaxSpeed If true, uses the robot's physical maximum speed from the drive subsystem.
   *     If false, uses the artificial MAX_CONTROL_SPEED limit for more controlled driving.
   * @return A command that drives the robot with turbo and optional face-target
   */
  public static Command joystickDriveWithTurboAndFaceTarget(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier turboSupplier,
      BooleanSupplier faceTargetEnabledSupplier,
      BooleanSupplier isRobotCentricSupplier,
      ProfiledPIDController faceTargetController,
      boolean usePhysicalMaxSpeed) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Square turbo input for quadratic response (always positive, so just square it)
          double turboInput = turboSupplier.getAsDouble();
          turboInput = turboInput * turboInput;

          // Determine speed limits based on parameter
          double maxLinearSpeed = usePhysicalMaxSpeed 
              ? drive.getMaxLinearSpeedMetersPerSec() 
              : MAX_CONTROL_SPEED;
          double maxAngularRate = usePhysicalMaxSpeed 
              ? drive.getMaxAngularSpeedRadPerSec() 
              : MAX_ANGULAR_RATE;

          // Apply turbo scaling to linear velocity components
          double velocityX = scaleAxisWithTurbo(linearVelocity.getY(), turboInput, maxLinearSpeed);  // Forward/backward joystick → field X
          double velocityY = scaleAxisWithTurbo(linearVelocity.getX(), turboInput, maxLinearSpeed);  // Left/right joystick → field Y

          // Determine rotational rate: use face-target PID if enabled, otherwise use joystick
          double rotationalRate;
          if (faceTargetEnabledSupplier.getAsBoolean()) {
            // Calculate target angle and use PID controller to rotate toward it
            rotationalRate = computeOmegaToFaceHub(drive, faceTargetController);
          } else {
            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), XBOX_JOYSTICK_DEADBAND);
            
            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega);
            
            // Apply turbo scaling to omega
            rotationalRate = scaleAxisWithTurbo(omega, turboInput, maxAngularRate);
            
            // Reset PID controller when not using face-target mode
            faceTargetController.reset(drive.getRotation().getRadians());
          }

          boolean robotCentric = isRobotCentricSupplier.getAsBoolean();
          Logger.recordOutput("Drive/CentricMode", robotCentric ? "Robot" : "Field");

          if (robotCentric) {
            drive.runVelocity(new ChassisSpeeds(velocityX, velocityY, rotationalRate));
          } else {
            driveFieldRelative(drive, velocityX, velocityY, rotationalRate);
          }
        },
        drive);
  } // End joystickDriveWithTurboAndFaceTarget

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              driveFieldRelative(
                  drive,
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  } // End joystickDriveAtAngle


  // ============================================================================
  // Characterization Commands
  // ============================================================================

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  } // End feedforwardCharacterization

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  } // End wheelRadiusCharacterization

  /** State for wheel radius characterization. */
  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  } // End WheelRadiusCharacterizationState


  // ============================================================================
  // Pathfinding Commands
  // ============================================================================

  /**
   * Returns a command that pathfinds to the start of the specified path, then follows it.
   * This is a modular method that can be used for any path.
   *
   * @param drive The drive subsystem
   * @param pathName The name of the path file (without .path extension) in deploy/pathplanner/paths/
   * @return A command that pathfinds then follows the path
   * @throws RuntimeException if the path file cannot be loaded
   */
  public static Command pathfindThenFollowPath(Drive drive, String pathName) {
    // Load the path from file with error handling
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      e.printStackTrace();
      throw new RuntimeException("Failed to load path: " + pathName + ". Make sure the path exists in deploy/pathplanner/paths/", e);
    }
    
    // Use the path's constraints for pathfinding
    PathConstraints pathfindingConstraints = path.getGlobalConstraints();
    
    // Return the AutoBuilder command
    return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
  } // End pathfindThenFollowPath

  // ============================================================================
  // Swerve Calibration Commands
  // ============================================================================
  public static void swerveCalibration(LoggedDashboardChooser<Command> autoChooser, Drive drive) {
		autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  } // End swerveCalibration
}
