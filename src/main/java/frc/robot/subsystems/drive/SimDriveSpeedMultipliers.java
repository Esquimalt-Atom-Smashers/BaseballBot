package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.simulation.SecondSimRobotOutputs;

import java.util.Random;
import org.littletonrobotics.junction.Logger;

/**
 * Independent per-robot random drive speed multipliers for sim (same range, distinct draws). Primary Talon+Maple and
 * Maple-direct each use a {@link Bank}.
 */
public final class SimDriveSpeedMultipliers {

  /** Primary sim ({@link ModuleIOSim}); logs multipliers once under {@code Subsystems/Drive/Sim/DriveSpeedMultipliers}. */
  public static final Bank TALON_MAPLE_SIM = new Bank("Subsystems/Drive/Sim/DriveSpeedMultipliers");

  /**
   * Maple-direct path ({@link ModuleIOSimMapleDirect}); logs once under namespaced key (same relative path as primary,
   * under {@link SecondSimRobotOutputs#LOG_ROOT_PREFIX}).
   */
  public static final Bank MAPLE_DIRECT =
      new Bank(SecondSimRobotOutputs.LOG_ROOT_PREFIX + "Subsystems/Drive/Sim/DriveSpeedMultipliers");

  public static final class Bank {
    final String logKey;
    private final Random random;
    private final Object lock = new Object();
    private final double[] multipliers = {1.0, 1.0, 1.0, 1.0};
    private boolean initialized;

    Bank(String logKey) {
      this.logKey = logKey;
      this.random = new Random();
    }

    void ensureInitialized() {
      synchronized (lock) {
        if (!initialized) {
          resampleDistinctMultipliers(multipliers, random);
          Logger.recordOutput(logKey, multipliers);
          initialized = true;
        }
      }
    }

    double get(int moduleIndex) {
      ensureInitialized();
      return multipliers[moduleIndex];
    }
  }

  private SimDriveSpeedMultipliers() {}

  /** First call samples multipliers and logs once under this bank’s key. */
  public static void ensureInitialized(Bank bank) {
    bank.ensureInitialized();
  }

  public static double get(Bank bank, int moduleIndex) {
    return bank.get(moduleIndex);
  }

  static void resampleDistinctMultipliers(double[] dest, Random random) {
    final double minMultiplier = Constants.SimulationDrive.kDriveSpeedMultiplierMin;
    final double maxMultiplier = Constants.SimulationDrive.kDriveSpeedMultiplierMax;
    final double multiplierRange = maxMultiplier - minMultiplier;
    double[] next = new double[4];
    outer:
    while (true) {
      for (int i = 0; i < 4; i++) {
        next[i] = minMultiplier + random.nextDouble() * multiplierRange;
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
    System.arraycopy(next, 0, dest, 0, 4);
  }
}
