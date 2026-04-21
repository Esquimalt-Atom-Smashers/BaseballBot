// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Zones;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Default drive command. */
public class TeleopDrive extends Command {
  private final Drive drive;
  private final CommandXboxController controller;
  private final BooleanSupplier isRobotCentricSupplier;
  private final BooleanSupplier isFacingHubSupplier;
  private final ProfiledPIDController faceTargetController;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private int flipFactor = 1;

  /** When true, joystick field axes are negated (red-alliance convention). */
  private final BooleanSupplier fieldFlipTreatAsRedAlliance;

  /** Prefix for {@link Logger} keys under {@code TeleopDrive/…}; empty string uses unprefixed paths. */
  private final String logRoot;

  private final Trigger inTrenchZoneTrigger;

  /// Disabled (Bump Zone is not used)
  // @AutoLogOutput
  // private final Trigger inBumpZoneTrigger;

  private final PIDController trenchYController =
      new PIDController(SwerveConstants.TRENCH_Y_KP, SwerveConstants.TRENCH_Y_KI, SwerveConstants.TRENCH_Y_KD);
  private final PIDController rotationController =
      new PIDController(SwerveConstants.ROTATION_KP, SwerveConstants.ROTATION_KI, SwerveConstants.ROTATION_KD);

  /** When true, set DriveMode to MANUAL_OVERRIDE (Manual Override). */
  private BooleanSupplier manualOverrideSupplier = () -> false;

  private DriveMode currentDriveMode = DriveMode.NORMAL;

  private ChassisSpeeds desiredFieldSpeeds = new ChassisSpeeds();

  /** Field flip follows {@link DriverStation} alliance; AdvantageKit keys are {@code TeleopDrive/…} (no path prefix). */
  public TeleopDrive(
      Drive drive,
      CommandXboxController controller,
      BooleanSupplier isRobotCentricSupplier,
      BooleanSupplier isFacingHubSupplier,
      ProfiledPIDController faceTargetController) {
    this(
        drive,
        controller,
        isRobotCentricSupplier,
        isFacingHubSupplier,
        faceTargetController,
        null,
        null);
  } // End TeleopDrive Constructor

  /**
   * @param fieldFlipTreatAsRedAlliance when non-null, supplies red-alliance field flip; otherwise {@link DriverStation} alliance is used.
   * @param logRoot prepended to AdvantageKit keys under {@code TeleopDrive/…}; may be empty.
   */
  public TeleopDrive(
      Drive drive,
      CommandXboxController controller,
      BooleanSupplier isRobotCentricSupplier,
      BooleanSupplier isFacingHubSupplier,
      ProfiledPIDController faceTargetController,
      BooleanSupplier fieldFlipTreatAsRedAlliance,
      String logRoot) {
    this.drive = drive;
    this.controller = controller;
    this.isRobotCentricSupplier = isRobotCentricSupplier;
    this.isFacingHubSupplier = isFacingHubSupplier;
    this.faceTargetController = faceTargetController;
    this.fieldFlipTreatAsRedAlliance = fieldFlipTreatAsRedAlliance != null ? fieldFlipTreatAsRedAlliance : TeleopDrive::driverStationIsRedAlliance;
    this.logRoot = logRoot != null ? logRoot : "";
    this.xSupplier = () -> -controller.getLeftY() * flipFactor;
    this.ySupplier = () -> -controller.getLeftX() * flipFactor;
    this.omegaSupplier = () -> -controller.getRightX();

    trenchYController.setTolerance(SwerveConstants.TRENCH_Y_TOLERANCE_M);
    rotationController.setTolerance(SwerveConstants.ROTATION_TOLERANCE_RAD);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    inTrenchZoneTrigger = Zones.TRENCH_ZONES
        .willContain(drive::getPose, drive::getFieldRelativeChassisSpeeds, Seconds.of(SwerveConstants.TRENCH_ALIGN_TIME_S))
        .debounce(0.1);

    /// Disabled (Bump Zone is not used)
    // inBumpZoneTrigger = Zones.BUMP_ZONES
    //     .willContain(drive::getPose, drive::getFieldRelativeChassisSpeeds, Seconds.of(SwerveConstants.BUMP_ALIGN_TIME_S))
    //     .debounce(0.1);

    inTrenchZoneTrigger.onTrue(Commands.runOnce(() -> currentDriveMode = DriveMode.TRENCH_LOCK));
    /// Disabled (Bump Zone is not used)
    // inBumpZoneTrigger.onTrue(Commands.runOnce(() -> currentDriveMode = DriveMode.BUMP_LOCK));
    // inTrenchZoneTrigger.or(inBumpZoneTrigger).onFalse(Commands.runOnce(() -> currentDriveMode = DriveMode.NORMAL));
    inTrenchZoneTrigger.onFalse(Commands.runOnce(() -> currentDriveMode = DriveMode.NORMAL));

    addRequirements(drive);
  } // End TeleopDrive Constructor

  private static boolean driverStationIsRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  } // End driverStationIsRedAlliance

  private static Translation2d getLinearVelocityFromJoysticks(double joystickX, double joystickY) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(joystickX, joystickY), ControllerConstants.CONTROLLER_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(joystickY, joystickX));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  } // End getLinearVelocityFromJoysticks

  private double getTrenchYMeters() {
    Pose2d robotPose = drive.getPose();
    if (robotPose.getY() >= FieldConstants.FIELD_WIDTH_M / 2.0) {
      return FieldConstants.FIELD_WIDTH_M - FieldConstants.TRENCH_CENTER_M;
    }
    return FieldConstants.TRENCH_CENTER_M;
  } // End getTrenchYMeters

  private Rotation2d getTrenchLockAngle() {
    if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees(), -180, 180)) < 90) {
      return Rotation2d.kZero;
    }
    return Rotation2d.k180deg;
  } // End getTrenchLockAngle

  private Rotation2d getBumpLockAngle() {
    for (int diagonalDeg = -135; diagonalDeg < 180; diagonalDeg += 90) {
      if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees() - diagonalDeg, -180, 180)) <= 45) {
        return Rotation2d.fromDegrees(diagonalDeg);
      }
    }
    return Rotation2d.kZero;
  } // End getBumpLockAngle

  /** When the supplier is true during {@link #execute}, drive mode is forced to manual override. */
  public void setManualOverrideSupplier(BooleanSupplier supplier) {
    manualOverrideSupplier = supplier != null ? supplier : () -> false;
  } // End setManualOverrideSupplier

  @Override
  public void initialize() {
    flipFactor = fieldFlipTreatAsRedAlliance.getAsBoolean() ? -1 : 1;
    trenchYController.reset();
    rotationController.reset();
  } // End initialize

  @Override
  public void execute() {
    // Right trigger (0 to 1) linearly scales speed from default to max
    double trigger = controller.getRightTriggerAxis();
    double maxDriveSpeedMps = SwerveConstants.DEFAULT_DRIVE_SPEED_MPS
        + (SwerveConstants.FAST_DRIVE_SPEED_MPS - SwerveConstants.DEFAULT_DRIVE_SPEED_MPS) * trigger;
    double maxRotSpeedRadPerS = SwerveConstants.DEFAULT_ROT_SPEED_RAD_PER_S
        + (SwerveConstants.FAST_ROT_SPEED_RAD_PER_S - SwerveConstants.DEFAULT_ROT_SPEED_RAD_PER_S) * trigger;

    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    linearVelocity = linearVelocity.times(maxDriveSpeedMps);

    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerConstants.CONTROLLER_DEADBAND);
    omega = Math.copySign(omega * omega, omega);

    this.desiredFieldSpeeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

    if (manualOverrideSupplier.getAsBoolean()) {
      currentDriveMode = DriveMode.MANUAL_OVERRIDE;
    }

    switch (currentDriveMode) {
      case MANUAL_OVERRIDE:
      case NORMAL:
        double vx = linearVelocity.getX();
        double vy = linearVelocity.getY();
        double rot = isFacingHubSupplier.getAsBoolean()
            ? DriveCommands.computeOmegaToFaceHub(drive, faceTargetController)
            : maxRotSpeedRadPerS * omega;

        // // When the robot is stationary, hold position with X-pattern base
        // if (Math.abs(vx) < 1e-3 && Math.abs(vy) < 1e-3 && Math.abs(rot) < 1e-3) {
        //   drive.stopWithX();
        //   break;
        // }

        if (isRobotCentricSupplier.getAsBoolean()) {
          drive.runVelocity(new ChassisSpeeds(vx, vy, rot));
        } else {
          drive.driveFieldCentric(vx, vy, rot);
        }
        break;
      case TRENCH_LOCK:
        trenchYController.setSetpoint(getTrenchYMeters());
        double yVel = trenchYController.calculate(drive.getPose().getY());
        if (trenchYController.atSetpoint()) {
          yVel = 0;
        }
        rotationController.setSetpoint(getTrenchLockAngle().getRadians());
        double rotSpeedToStraight = rotationController.calculate(drive.getRotation().getRadians());
        if (rotationController.atSetpoint()) {
          rotSpeedToStraight = 0;
        }
        drive.driveFieldCentric(
            linearVelocity.getX(),
            yVel,
            rotSpeedToStraight);
        break;
      case BUMP_LOCK:
        rotationController.setSetpoint(getBumpLockAngle().getRadians());
        double rotSpeedToDiagonal = rotationController.calculate(drive.getRotation().getRadians());
        if (rotationController.atSetpoint()) {
          rotSpeedToDiagonal = 0;
        }
        drive.driveFieldCentric(
            linearVelocity.getX(),
            linearVelocity.getY(),
            rotSpeedToDiagonal);
        break;
    }

    Logger.recordOutput(logRoot + "TeleopDrive/inTrenchZoneTrigger", inTrenchZoneTrigger.getAsBoolean());
    Logger.recordOutput(logRoot + "TeleopDrive/currentDriveMode", currentDriveMode.toString());
    Logger.recordOutput(logRoot + "TeleopDrive/TargetFieldRelativeSpeeds", desiredFieldSpeeds);
  } // End execute

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  } // End isFinished

  private enum DriveMode {
    NORMAL,
    TRENCH_LOCK,
    BUMP_LOCK,
    MANUAL_OVERRIDE
  } // End DriveMode
}
