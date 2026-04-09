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
 * Simulation-only coordinator for shooter fuel: tracks stored fuel when field fuel sim is on, gates
 * intake by capacity, launches into {@link FuelSim} when enabled, and drives ghost-shot logging when not.
 */
public class ShooterSim {

  private static final int CAPACITY = 30;
  private static final double SHOOT_INTERVAL_SEC = 0.25;

  private final FuelSim fuelSim;
  private final boolean launchSpawnsInFuelSim;
  /** When fuel sim is off, logs a flying ghost ball; may be null. */
  private final ShooterSimVisualizer ghostFuelVisualizer;
  private int fuelStored = 8;
  private final Timer shootTimer = new Timer();

  public ShooterSim(
      FuelSim fuelSim, boolean launchSpawnsInFuelSim, ShooterSimVisualizer ghostFuelVisualizer) {
    this.fuelSim = fuelSim;
    this.launchSpawnsInFuelSim = launchSpawnsInFuelSim;
    this.ghostFuelVisualizer = ghostFuelVisualizer;
    shootTimer.start();
  } // End ShooterSim Constructor

  /** Stored fuel count, or {@link #CAPACITY} when field fuel sim is off (infinite-ammo mode for logging). */
  public int getFuelStored() {
    return launchSpawnsInFuelSim ? fuelStored : CAPACITY;
  } // End getFuelStored

  /** True when the Shooter can accept more fuel (for FuelSim intake gating). */
  public boolean canIntake() {
    return launchSpawnsInFuelSim && fuelStored < CAPACITY;
  } // End canIntake

  /** Called by FuelSim when a ball is intaked; increments stored count up to {@link #CAPACITY}. */
  public void intakeFuel() {
    if (fuelStored < CAPACITY) {
      fuelStored++;
    }
  } // End intakeFuel

  /**
   * Launch one fuel using current Turret, Hood, and Flywheel. With field fuel sim: requires stored
   * fuel, decrements, spawns in FuelSim. Without: no decrement (infinite), optional ghost visual only.
   */
  public void launchFuel(Turret turret, Hood hood, Flywheel flywheel) {
    double turretYawRad = turret.getRobotFramePosition().getRadians();
    double hoodAngleRad = hood.getAngleRad();
    double flywheelSurfaceMps = flywheel.getTargetVelocityRadPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
    double ballExitVelMps = flywheelSurfaceMps * ShooterConstants.kFlywheelSurfaceDivider;

    if (launchSpawnsInFuelSim) {
      if (fuelStored == 0) return;
      fuelStored--;
      double elevationRad = Math.PI / 2 - hoodAngleRad;
      fuelSim.launchFuel(
          MetersPerSecond.of(ballExitVelMps),
          Radians.of(elevationRad),
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
   * Call from simulation periodic. Launches fuel when timer elapsed, enabled, Shooter ready, and shooting active.
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
