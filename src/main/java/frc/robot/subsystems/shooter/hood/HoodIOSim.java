package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;

/**
 * Hood IO using a position servo: commanded elevation is rate-limited.
 */
public class HoodIOSim implements HoodIO {

  private static final double kLoopPeriodSecs = 0.02;

  private double targetPositionRad = kDisabledAngleRad;
  private double currentPositionRad = kDisabledAngleRad;
  private boolean isStopped = false;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (!isStopped) {
      double errorRad = targetPositionRad - currentPositionRad;
      double maxStepRad = kSimMaxSlewRadPerSec * kLoopPeriodSecs;
      double stepRad = MathUtil.clamp(errorRad, -maxStepRad, maxStepRad);
      currentPositionRad += stepRad;
      inputs.velocityRadsPerSec = stepRad / kLoopPeriodSecs;
    } else {
      inputs.velocityRadsPerSec = 0.0;
    }

    inputs.motorConnected = true;
    inputs.positionRads = currentPositionRad;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetPositionRad) {
    this.targetPositionRad = targetPositionRad;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void resetEncoder() {
    currentPositionRad = kDisabledAngleRad;
    targetPositionRad = kDisabledAngleRad;
  } // End resetEncoder

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
