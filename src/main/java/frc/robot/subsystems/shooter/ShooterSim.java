// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at the root of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

import frc.robot.simulation.FuelSim;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

/**
 * Simulation-only fuel bookkeeping and launch: stored count with {@link #CAPACITY}, optional {@link FuelSim} spawn, or
 * optional {@link ShooterSimVisualizer} when not spawning in {@link FuelSim}.
 */
public class ShooterSim {

  private static final int CAPACITY = 30;
  public static final int kInitialFuelStored = 8;
  private static final double SHOOT_INTERVAL_SEC = 0.25;

  private final FuelSim fuelSim;
  private final int fuelRobotIndex;
  private final boolean launchSpawnsInFuelSim;
  /** Optional; used when {@link #launchSpawnsInFuelSim} is false; may be {@code null}. */
  private final ShooterSimVisualizer ghostFuelVisualizer;
  private int fuelStored = kInitialFuelStored;
  private final Timer shootTimer = new Timer();

  /** @param fuelRobotIndex {@link FuelSim} registered-robot index for {@link FuelSim#launchFuel} */
  public ShooterSim(
      FuelSim fuelSim,
      int fuelRobotIndex,
      boolean launchSpawnsInFuelSim,
      ShooterSimVisualizer ghostFuelVisualizer) {
    this.fuelSim = fuelSim;
    this.fuelRobotIndex = fuelRobotIndex;
    this.launchSpawnsInFuelSim = launchSpawnsInFuelSim;
    this.ghostFuelVisualizer = ghostFuelVisualizer;
    shootTimer.start();
    if (launchSpawnsInFuelSim) {
      fuelSim.setCarriedFuelCount(fuelRobotIndex, fuelStored);
      fuelSim.registerShooterFuelReset(this::resetFuelStored);
    }
  } // End ShooterSim Constructor

  /** Sets {@link #fuelStored} to the initial sim count (used when {@link FuelSim#resetFuel()} runs). */
  public void resetFuelStored() {
    fuelStored = kInitialFuelStored;
    if (launchSpawnsInFuelSim) {
      fuelSim.setCarriedFuelCount(fuelRobotIndex, fuelStored);
    }
  } // End resetFuelStored

  /** @return {@link #fuelStored} when {@link #launchSpawnsInFuelSim}, otherwise {@link #CAPACITY} */
  public int getFuelStored() {
    return launchSpawnsInFuelSim ? fuelStored : CAPACITY;
  } // End getFuelStored

  /** @return true when {@link #launchSpawnsInFuelSim} and {@link #fuelStored} &lt; {@link #CAPACITY} */
  public boolean canIntake() {
    return launchSpawnsInFuelSim && fuelStored < CAPACITY;
  } // End canIntake

  /** Increments {@link #fuelStored} up to {@link #CAPACITY}. */
  public void intakeFuel() {
    if (fuelStored < CAPACITY) {
      fuelStored++;
    }
  } // End intakeFuel

  /**
   * Computes exit speed from flywheel target and hood/turret aim, then either {@link FuelSim#launchFuel} (decrementing
   * {@link #fuelStored} when {@link #launchSpawnsInFuelSim}) or {@link ShooterSimVisualizer#startGhostFuel} when
   * {@link #ghostFuelVisualizer} is non-null.
   */
  public void launchFuel(Turret turret, Hood hood, Flywheel flywheel) {
    double turretYawRad = turret.getRobotFramePosition().getRadians();
    double hoodAngleRad = hood.getAngleRad();
    double flywheelSurfaceMps = flywheel.getVelocityRadPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
    double ballExitVelMps = flywheelSurfaceMps 
        * ShooterConstants.kFlywheelSurfaceDivider
        * ShooterConstants.kSimFlywheelToFuelExitVelocityEfficiency;

    if (launchSpawnsInFuelSim) {
      if (fuelStored == 0) return;
      fuelStored--;
      fuelSim.launchFuel(
          fuelRobotIndex,
          MetersPerSecond.of(ballExitVelMps),
          Radians.of(hoodAngleRad),
          Radians.of(turretYawRad),
          ShooterConstants.robotToTurret);
    } else if (ghostFuelVisualizer != null) {
      ghostFuelVisualizer.startGhostFuel(
          MetersPerSecond.of(ballExitVelMps),
          Radians.of(hoodAngleRad),
          Radians.of(turretYawRad));
    }
  } // End launchFuel

  /**
   * When {@link #shootTimer} has passed {@link #SHOOT_INTERVAL_SEC}, DS enabled, {@link Shooter#isReadyToShoot()}, and
   * {@code isShootingActive}, runs {@link #launchFuel}.
   */
  public void update(
      Shooter shooter,
      BooleanSupplier isShootingActive,
      Turret turret,
      Hood hood,
      Flywheel flywheel) {
    if (shootTimer.advanceIfElapsed(SHOOT_INTERVAL_SEC)
        && DriverStation.isEnabled()
        && shooter.isReadyToShoot()
        && isShootingActive.getAsBoolean()) {
      launchFuel(turret, hood, flywheel);
    }
  } // End update
}
