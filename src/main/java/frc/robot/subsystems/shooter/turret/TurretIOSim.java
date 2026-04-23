package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.kSimMaxTurretRadPerSec;

import edu.wpi.first.math.MathUtil;

/** Turret IO for simulation; rate-limited setpoint following. */
public class TurretIOSim implements TurretIO {

  private static final double kLoopPeriodSecs = 0.02;

  private double targetPositionRad = 0.0;
  private double velocityFeedforwardRadPerSec = 0.0;
  private double currentPositionRad = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (!isStopped) {
      double errorRad = targetPositionRad - currentPositionRad;
      // Single rate cap on FF + discrete-time error rate (old model capped only the P step, not FF+ P).
      double combinedVelRadPerSec =
          MathUtil.clamp(
              velocityFeedforwardRadPerSec + errorRad / kLoopPeriodSecs,
              -kSimMaxTurretRadPerSec,
              kSimMaxTurretRadPerSec);
      currentPositionRad += combinedVelRadPerSec * kLoopPeriodSecs;

      inputs.velocityRadsPerSec = combinedVelRadPerSec;
    } else {
      inputs.velocityRadsPerSec = 0.0;
    }

    inputs.motorConnected = true;
    inputs.positionRads = currentPositionRad;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    targetPositionRad = targetRads;
    velocityFeedforwardRadPerSec = 0.0;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void setTargetPosition(double targetRads, double velocityFeedforwardRadPerSec) {
    targetPositionRad = targetRads;
    this.velocityFeedforwardRadPerSec = velocityFeedforwardRadPerSec;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
