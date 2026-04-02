// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  /** Nominal bus voltage for voltage compensation. */
  public static final double kNominalVoltage = 12.0;

  public static class Dimensions {
    public static final Distance BUMPER_THICKNESS = Inches.of(3.0); // Frame to edge of bumper
    public static final Distance BUMPER_HEIGHT = Inches.of(5.995); // Height from floor to top of bumper
    public static final Distance FRAME_SIZE_X = Inches.of(26.0); // Front to back (x-axis)
    public static final Distance FRAME_SIZE_Y = Inches.of(28.0); // Left to right (y-axis)

    public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
    public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));

    /**
     * Protrusion past bumpers along robot +X when the extender is fully extended. MapleSim / dyn4j
     * chassis collision only; FuelSim uses {@link #FULL_LENGTH} for the robot box.
     */
    public static final double kExtensionPastBumpersMeters = Inches.of(8.696850).in(Meters);
  }

  /** Controller deadband and rumble. */
  public static class ControllerConstants {
    public static final double CONTROLLER_DEADBAND = 0.15;
    public static final double CONTROLLER_RUMBLE = 0.3;
  }

  /** Match timing. */
  public static class MatchTiming {
    private MatchTiming() {}

    /** Driver hang endgame period. */
    public static final double HANG_DRIVER_ENDGAME_SECONDS = 30.0;
  }

  /** Teleop drive speeds and trench/bump alignment. */
  public static class SwerveConstants { // XXX: Tune SwerveConstants, especially TRENCH_ALIGN_TIME_S
    public static final double DEFAULT_DRIVE_SPEED_MPS = 1.6;
    public static final double DEFAULT_ROT_SPEED_RAD_PER_S = 0.75 * 2 * Math.PI;
    /** From Drive.getMaxLinearSpeedMetersPerSec() (TunerConstants.kSpeedAt12Volts = 4.58 m/s). */
    public static final double FAST_DRIVE_SPEED_MPS = 4.58;
    /** From Drive.getMaxAngularSpeedRadPerSec(): max linear / drive base radius. Radius = hypot(9.75, 10.75) in → 0.36866 m. */
    public static final double FAST_ROT_SPEED_RAD_PER_S = 4.58 / 0.36866;
    public static final double TRENCH_ALIGN_TIME_S = 0.5;
    public static final double BUMP_ALIGN_TIME_S = 0.3;

    /** Trench Y (translation) PID and tolerance. */
    public static final double TRENCH_Y_KP = 8.0;
    public static final double TRENCH_Y_KI = 0.0;
    public static final double TRENCH_Y_KD = 0.05;
    public static final double TRENCH_Y_TOLERANCE_M = 0.127;

    /** Rotation (trench/bump lock) PID and tolerance. */
    public static final double ROTATION_KP = 5.0;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;
    public static final double ROTATION_TOLERANCE_RAD = Units.degreesToRadians(10.0);
  }

  /**
   * Drive realism for Maple / {@link frc.robot.subsystems.drive.ModuleIOSim} only (that IO is not
   * used on the real robot).
   */
  public static class SimulationDrive {
    /** Per-module closed-loop drive velocity scale; four wheels get four distinct values in range. */
    public static final double kDriveSpeedMultiplierMin = 0.97;
    public static final double kDriveSpeedMultiplierMax = 1.0;
  }

  /** Runtime mode. */
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
