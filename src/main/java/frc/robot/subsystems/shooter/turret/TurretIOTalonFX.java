package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

/** Turret IO using Talon FX with onboard position control. */
public class TurretIOTalonFX implements TurretIO {

  private final TalonFX motor;
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  private double lastP = kP;
  private double lastI = kI;
  private double lastD = kD;

  public TurretIOTalonFX() {
    motor = new TalonFX(kMotorId, TunerConstants.kCANBus);

    var talonFxConfig = new TalonFXConfiguration();
    talonFxConfig.MotorOutput.NeutralMode = kNeutralMode;
    talonFxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFxConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;
    talonFxConfig.Slot0.kP = kP;
    talonFxConfig.Slot0.kI = kI;
    talonFxConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> motor.getConfigurator().apply(talonFxConfig, 0.25));
  } // End TurretIOTalonFX Constructor

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double p = SmartDashboard.getNumber("Turret/kP", kP);
    double i = SmartDashboard.getNumber("Turret/kI", kI);
    double d = SmartDashboard.getNumber("Turret/kD", kD);

    if (p != lastP || i != lastI || d != lastD) {
      lastP = p;
      lastI = i;
      lastD = d;

      var slot0 = new Slot0Configs().withKP(p).withKI(i).withKD(d);
      tryUntilOk(5, () -> motor.getConfigurator().apply(slot0, 0.25));
    }
    
    var signalRefreshStatus =
        BaseStatusSignal.refreshAll(
            motor.getPosition(),
            motor.getVelocity(),
            motor.getMotorVoltage(),
            motor.getSupplyCurrent());
     
    inputs.motorConnected = signalRefreshStatus.equals(StatusCode.OK);
    double motorShaftRotations = motor.getPosition().getValueAsDouble();
    inputs.positionRads = Units.rotationsToRadians(motorShaftRotations) / kGearRatio;
    double motorShaftRps = motor.getVelocity().getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(motorShaftRps) / kGearRatio;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    setTargetPosition(targetRads, 0.0);
  } // End setTargetPosition

  @Override
  public void setTargetPosition(double targetRads, double velocityFeedforwardRadPerSec) {
    setTargetPosition(targetRads, velocityFeedforwardRadPerSec, 0.0);
  } // End setTargetPosition

  @Override
  public void setTargetPosition(
      double targetRads, double velocityFeedforwardRadPerSec, double velocityFFVolts) {
    double targetRotations = Units.radiansToRotations(targetRads);
    double motorTargetRotations = targetRotations * kGearRatio;
    // Motor velocity (rot/s) = Turret rad/s * gearRatio / (2*pi)
    double motorVelRotPerSec =
        Units.radiansToRotations(velocityFeedforwardRadPerSec) * kGearRatio;
    motor.setControl(
        positionVoltageRequest
            .withPosition(motorTargetRotations)
            .withVelocity(motorVelRotPerSec)
            .withFeedForward(velocityFFVolts));
  } // End setTargetPosition

  @Override
  public void resetEncoder() { 
    motor.setPosition(0);
  }

  @Override
  public void stop() {
    motor.setControl(new NeutralOut());
  } // End stop
}
