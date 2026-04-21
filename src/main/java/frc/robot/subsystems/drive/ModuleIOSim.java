// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import java.util.Random;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * Physics sim implementation of module IO using MapleSim. The sim models are configured using a set
 * of module constants from Phoenix. MapleSim wraps the actual TalonFX controllers, so commands go
 * through the same code path as real hardware.
 */
public class ModuleIOSim implements ModuleIO {
  private static final Object driveMultiplierLock = new Object();
  private static final double[] driveSpeedMultipliers = {1.0, 1.0, 1.0, 1.0};
  private static boolean driveSpeedMultipliersInitialized = false;
  private static final Random driveMultiplierRandom = new Random();

  private final int moduleIndex;

  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

  // Hardware objects (simulated by MapleSim)
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // MapleSim simulation
  private final SwerveModuleSimulation simulation;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
      SwerveModuleSimulation simulation,
      int moduleIndex) {
    this(constants, simulation, moduleIndex, TunerConstants.kCANBus, 0);
  } // End ModuleIOSim Constructor

  /** @param canBus identifier passed to {@link TalonFX} and {@link CANcoder} constructors */
  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
      SwerveModuleSimulation simulation,
      int moduleIndex,
      CANBus canBus) {
    this(constants, simulation, moduleIndex, canBus, 0);
  } // End ModuleIOSim Constructor

  /**
   * @param canBus identifier passed to {@link TalonFX} and {@link CANcoder} constructors
   * @param deviceIdOffset added to {@code DriveMotorId}, {@code SteerMotorId}, and {@code EncoderId} from {@code constants}
   */
  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
      SwerveModuleSimulation simulation,
      int moduleIndex,
      CANBus canBus,
      int deviceIdOffset) {
    if (moduleIndex < 0 || moduleIndex > 3) {
      throw new IllegalArgumentException("moduleIndex must be 0..3, got " + moduleIndex);
    }
    this.moduleIndex = moduleIndex;
    this.constants = PhoenixUtil.regulateModuleConstantForSimulation(constants);
    this.simulation = simulation;
    ensureDriveSpeedMultipliersInitialized();

    int driveMotorId = this.constants.DriveMotorId + deviceIdOffset;
    int steerMotorId = this.constants.SteerMotorId + deviceIdOffset;
    int encoderId = this.constants.EncoderId + deviceIdOffset;

    driveTalon = new TalonFX(driveMotorId, canBus);
    turnTalon = new TalonFX(steerMotorId, canBus);
    cancoder = new CANcoder(encoderId, canBus);

    // Configure drive motor
    var driveConfig = this.constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = this.constants.DriveMotorGains;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = this.constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -this.constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = this.constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        this.constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor with simulation-specific gains
    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = this.constants.SteerMotorGains;
    // Sim-only: regulated steer gains use high kD; with Maple+Talon sim that hunts and shows up as
    // chassis shake / curved drift. Softer kD and kS=0 here stabilizes azimuth without changing real-robot tuning.
    turnConfig.Slot0.withKD(0.5).withKS(0);

    turnConfig.Feedback.FeedbackRemoteSensorID = encoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (this.constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new RuntimeException(
              "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOSim. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
    turnConfig.Feedback.RotorToSensorRatio = this.constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / this.constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * this.constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        this.constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = this.constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = this.constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        this.constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);

    // Wire up MapleSim to control the simulated TalonFX controllers
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));
    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
  } // End ModuleIOSim Constructor

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals from simulated controllers
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / constants.DriveMotorGearRatio;
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / constants.DriveMotorGearRatio;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // Update odometry inputs from MapleSim (high-frequency cached positions)
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double scaledRadPerSec = velocityRadPerSec * driveSpeedMultipliers[moduleIndex];
    double velocityRotPerSec =
        Units.radiansToRotations(scaledRadPerSec) * constants.DriveMotorGearRatio;
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  } // End setDriveVelocity

  private static void ensureDriveSpeedMultipliersInitialized() {
    synchronized (driveMultiplierLock) {
      if (!driveSpeedMultipliersInitialized) {
        resampleDistinctDriveSpeedMultipliers();
        driveSpeedMultipliersInitialized = true;
        Logger.recordOutput("Subsystems/Drive/Sim/DriveSpeedMultipliers", driveSpeedMultipliers);
      }
    }
  } // End ensureDriveSpeedMultipliersInitialized

  /**
   * Fills {@link #driveSpeedMultipliers} with four distinct values in
   * [{@link Constants.SimulationDrive#kDriveSpeedMultiplierMin},
   * {@link Constants.SimulationDrive#kDriveSpeedMultiplierMax}].
   */
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

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(rotation.getRotations());
        });
  }
}
