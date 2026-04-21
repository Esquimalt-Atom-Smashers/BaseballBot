// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

/** {@link ModuleIO} for MapleSim using {@link SimulatedMotorController} hooks only (no TalonFX/CANcoder device sim). */
public class ModuleIOSimMapleDirect implements ModuleIO {
  private static final Object driveMultiplierLock = new Object();
  private static final double[] driveSpeedMultipliers = {1.0, 1.0, 1.0, 1.0};
  private static boolean driveSpeedMultipliersInitialized = false;
  private static final Random driveMultiplierRandom = new Random();

  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;
  private final SwerveModuleSimulation simulation;
  private final MapleDirectDriveController driveController;
  private final MapleDirectSteerController steerController;

  public ModuleIOSimMapleDirect(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
      SwerveModuleSimulation simulation,
      int moduleIndex) {
    if (moduleIndex < 0 || moduleIndex > 3) {
      throw new IllegalArgumentException("moduleIndex must be 0..3, got " + moduleIndex);
    }
    if (constants.DriveMotorClosedLoopOutput != ClosedLoopOutputType.Voltage || constants.SteerMotorClosedLoopOutput != ClosedLoopOutputType.Voltage) {
      throw new UnsupportedOperationException(
          "ModuleIOSimMapleDirect only supports Voltage closed-loop output.");
    }
    this.constants = PhoenixUtil.regulateModuleConstantForSimulation(constants);
    this.simulation = simulation;
    ensureDriveSpeedMultipliersInitialized();
    driveController = new MapleDirectDriveController(this.constants, moduleIndex);
    steerController = new MapleDirectSteerController(this.constants);
    simulation.useDriveMotorController(driveController);
    simulation.useSteerMotorController(steerController);
  } // End ModuleIOSimMapleDirect Constructor

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.drivePositionRad = simulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = simulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = simulation.getDriveMotorAppliedVoltage().in(Volts);
    inputs.driveCurrentAmps = simulation.getDriveMotorStatorCurrent().in(Amps);

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    Rotation2d facing = simulation.getSteerAbsoluteFacing();
    inputs.turnAbsolutePosition = facing;
    inputs.turnPosition = facing;
    inputs.turnVelocityRadPerSec = simulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = simulation.getSteerMotorAppliedVoltage().in(Volts);
    inputs.turnCurrentAmps = simulation.getSteerMotorStatorCurrent().in(Amps);

    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  } // End updateInputs

  @Override
  public void setDriveOpenLoop(double output) {
    driveController.setOpenLoop(output);
  } // End setDriveOpenLoop

  @Override
  public void setTurnOpenLoop(double output) {
    steerController.setOpenLoop(output);
  } // End setTurnOpenLoop

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveController.setVelocityClosedLoop(velocityRadPerSec);
  } // End setDriveVelocity

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    steerController.setPositionClosedLoop(rotation);
  } // End setTurnPosition

  private static void ensureDriveSpeedMultipliersInitialized() {
    synchronized (driveMultiplierLock) {
      if (!driveSpeedMultipliersInitialized) {
        resampleDistinctDriveSpeedMultipliers();
        driveSpeedMultipliersInitialized = true;
        Logger.recordOutput(
            "Subsystems/Drive/Sim/SecondRobotDriveSpeedMultipliers", driveSpeedMultipliers);
      }
    }
  } // End ensureDriveSpeedMultipliersInitialized

  private static void resampleDistinctDriveSpeedMultipliers() {
    final double lo = Constants.SimulationDrive.kDriveSpeedMultiplierMin;
    final double hi = Constants.SimulationDrive.kDriveSpeedMultiplierMax;
    final double span = hi - lo;
    double[] next = new double[4];
    outer:
    while (true) {
      for (int i = 0; i < 4; i++) {
        next[i] = lo + driveMultiplierRandom.nextDouble() * span;
      }
      for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
          if (Math.abs(next[i] - next[j]) < 1e-9) {
            continue outer;
          }
        }
      }
      break;
    }
    System.arraycopy(next, 0, driveSpeedMultipliers, 0, 4);
  } // End resampleDistinctDriveSpeedMultipliers

  private static final class MapleDirectDriveController implements SimulatedMotorController {
    /**
     * TalonFX velocity loops run on the CTRE stack; this Java PID runs every Maple sub-tick (~250 Hz) against
     * floor-coupled encoder velocity. Copying kD from tuner gains tends to amplify noise and rail voltage → runaway
     * in sim. Use a lighter P-only loop plus the same kV/kS feedforward as hardware.
     */
    private static final double MAPLE_DIRECT_KP_SCALE = 0.35;

    private final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        constants;
    private final int moduleIndex;
    private final PIDController pid;
    private final double kV;
    private final double kS;
    private boolean velocityMode = true;
    private double velocitySetpointRotPerSec;
    private double openLoopVolts;

    MapleDirectDriveController(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
        int moduleIndex) {
      this.constants = constants;
      this.moduleIndex = moduleIndex;
      var driveGains = constants.DriveMotorGains;
      double dt = SimulatedArena.getSimulationDt().in(Seconds);
      this.pid = new PIDController(driveGains.kP * MAPLE_DIRECT_KP_SCALE, 0.0, 0.0, dt);
      this.kV = driveGains.kV;
      this.kS = driveGains.kS;
    } // End MapleDirectDriveController Constructor

    void setOpenLoop(double output) {
      if (velocityMode) {
        pid.reset();
      }
      velocityMode = false;
      openLoopVolts = output;
    } // End setOpenLoop

    void setVelocityClosedLoop(double velocityRadPerSec) {
      if (!velocityMode) {
        pid.reset();
      }
      velocityMode = true;
      double scaledRadPerSec = velocityRadPerSec * driveSpeedMultipliers[moduleIndex];
      velocitySetpointRotPerSec = Units.radiansToRotations(scaledRadPerSec) * constants.DriveMotorGearRatio;
    } // End setVelocityClosedLoop

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      if (!velocityMode) {
        return Volts.of(MathUtil.clamp(openLoopVolts, -12.0, 12.0));
      }
      double velocityRotPerSecMeasured = encoderVelocity.in(RotationsPerSecond);
      double kSterm =
          Math.abs(velocitySetpointRotPerSec) > 1e-6
              ? kS * Math.signum(velocitySetpointRotPerSec)
              : 0.0;
      pid.setSetpoint(velocitySetpointRotPerSec);
      double appliedVolts = pid.calculate(velocityRotPerSecMeasured) + kV * velocitySetpointRotPerSec + kSterm;
      return Volts.of(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    } // End updateControlSignal
  } // End MapleDirectDriveController

  private static final class MapleDirectSteerController implements SimulatedMotorController {
    private final double kP;
    private boolean positionMode = true;
    private double positionSetpointRotations;
    private double openLoopVolts;

    MapleDirectSteerController(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
      var steerGains = constants.SteerMotorGains;
      this.kP = steerGains.kP;
      // P-only on wrapped rotation error; no kD at Maple sub-tick rate (avoids over-damping vs CTRE discrete loop).
    } // End MapleDirectSteerController Constructor

    void setOpenLoop(double output) {
      positionMode = false;
      openLoopVolts = output;
    } // End setOpenLoop

    void setPositionClosedLoop(Rotation2d rotation) {
      positionMode = true;
      positionSetpointRotations = rotation.getRotations();
    } // End setPositionClosedLoop

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      if (!positionMode) {
        return Volts.of(MathUtil.clamp(openLoopVolts, -12.0, 12.0));
      }
      Rotation2d currentAngle = Rotation2d.fromRotations(mechanismAngle.in(Rotations));
      Rotation2d setpointAngle = Rotation2d.fromRotations(positionSetpointRotations);
      double errorRotations = setpointAngle.minus(currentAngle).getRotations();
      double appliedVolts = kP * errorRotations;
      return Volts.of(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    } // End updateControlSignal
  } // End MapleDirectSteerController
}
