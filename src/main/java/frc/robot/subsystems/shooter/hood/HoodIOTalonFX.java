package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Hood IO using Talon FX with onboard position control. */
public class HoodIOTalonFX implements HoodIO {

  private final TalonFX motor;
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  public HoodIOTalonFX() {
    motor = new TalonFX(kMotorId, TunerConstants.kCANBus);

    var talonFxConfig = new TalonFXConfiguration();
    talonFxConfig.MotorOutput.NeutralMode = kNeutralMode;
    talonFxConfig.MotorOutput.Inverted =
      kMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    talonFxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFxConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;

    talonFxConfig.Slot0.kP = kP;
    talonFxConfig.Slot0.kI = kI;
    talonFxConfig.Slot0.kD = kD;
    
    tryUntilOk(5, () -> motor.getConfigurator().apply(talonFxConfig, 0.25));
  } // End HoodIOTalonFX Constructor

  @Override
  public void updateInputs(HoodIOInputs inputs) {
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
    double targetRotations = Units.radiansToRotations(targetRads);
    double motorTargetRotations = targetRotations * kGearRatio;
    motor.setControl(positionVoltageRequest.withPosition(motorTargetRotations));
  } // End setTargetPosition

  @Override
  public void resetEncoder() {
    motor.setPosition(0);
  } // End resetEncoder

  @Override
  public void stop() {
    motor.setControl(new NeutralOut());
  } // End stop
}
