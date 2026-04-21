package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.kStoredPositionMeters;
import static frc.robot.subsystems.hang.HangConstants.kSimPositionRampRateInchesPerSec;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

/** Hang IO for simulation; rate-limited setpoint following. */
public class HangIOSim implements HangIO {

  private static final double kLoopPeriodSecs = 0.02;
  private static final double kMaxMetersPerSec = Units.inchesToMeters(kSimPositionRampRateInchesPerSec);

  private double targetPositionMeters = 0.0;
  private double currentPositionMeters = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(HangIOInputs inputs) {
    if (!isStopped) {
      double errorMeters = targetPositionMeters - currentPositionMeters;
      double maxStepMeters = kMaxMetersPerSec * kLoopPeriodSecs;
      double stepMeters = MathUtil.clamp(errorMeters, -maxStepMeters, maxStepMeters);
      currentPositionMeters += stepMeters;
      inputs.velocityMetersPerSec = stepMeters / kLoopPeriodSecs;
    } else {
      inputs.velocityMetersPerSec = 0.0;
    }

    inputs.motorConnected = true;
    inputs.positionMeters = currentPositionMeters;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetMeters) {
    targetPositionMeters = targetMeters;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void resetEncoders() {
    currentPositionMeters = kStoredPositionMeters;
  } // End resetEncoders

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}
