package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Turret IO using a single SPARK MAX (NEO 550) with onboard position control. */
public class TurretIOSparkMax implements TurretIO {

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  private double lastP = kP;
  private double lastI = kI;
  private double lastD = kD;

  public TurretIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig
        .encoder
        .positionConversionFactor(1.0 / kGearRatio)
        .velocityConversionFactor(1.0 / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD);
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(kSignalsPeriodMs)
        .busVoltagePeriodMs(kSignalsPeriodMs)
        .outputCurrentPeriodMs(kSignalsPeriodMs)
        .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
        .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motor.setPeriodicFrameTimeout(0);
    SmartDashboard.putNumber("Turret/kAimFfVPerRadS", kAimFfVPerRadS);
  } // End TurretIOSparkMax Constructor

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double p = SmartDashboard.getNumber("Turret/kP", kP);
    double i = SmartDashboard.getNumber("Turret/kI", kI);
    double d = SmartDashboard.getNumber("Turret/kD", kD);

    if (p != lastP || i != lastI || d != lastD) {
      lastP = p;
      lastI = i;
      lastD = d;

      var sparkMaxConfig = new SparkMaxConfig();
      sparkMaxConfig.closedLoop.p(p).i(i).d(d);
      sparkMaxConfig.signals
          .appliedOutputPeriodMs(kSignalsPeriodMs)
          .busVoltagePeriodMs(kSignalsPeriodMs)
          .outputCurrentPeriodMs(kSignalsPeriodMs)
          .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
          .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
      motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      motor.setPeriodicFrameTimeout(0);
    }
    
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(encoder.getVelocity() / 60.0);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    setTargetPosition(targetRads, 0.0);
  } // End setTargetPosition

  @Override
  public void setTargetPosition(double targetRads, double velocityFeedforwardRadPerSec) {
    double targetRot = Units.radiansToRotations(targetRads);
    double kFf = SmartDashboard.getNumber("Turret/kAimFfVPerRadS", kAimFfVPerRadS);
    double arbFfVolts = MathUtil.clamp(velocityFeedforwardRadPerSec * kFf, -kMaxVoltage, kMaxVoltage);
    closedLoopController.setSetpoint(targetRot, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFfVolts);
  } // End setTargetPosition
  
  @Override
  public void resetEncoder() { 
    encoder.setPosition(0.0);
  } // End resetEncoder

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}
