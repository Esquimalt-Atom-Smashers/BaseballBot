// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.awt.GraphicsEnvironment;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.SimMatchTimeCache;
import frc.robot.simulation.TeamSignDisplayUtil;
import frc.robot.simulation.TeamSignSimWindow;
import frc.robot.util.HubShiftUtil;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  // SIM-only match countdown (see startSimMatchClock, simulationPeriodic).
  private boolean simMatchClockRunning = false;
  private double simMatchDurationSec = 0.0;
  private double simMatchStartTimestamp = 0.0;
  /** When true, hitting 0 on the sim clock disables the robot (end-of-auto behavior). */
  private boolean simDisableRobotWhenCountdownEnds = false;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing - only impacts real robot, not little to no effect on simulation
		// (see the template project documentation for details) 
    // Do NOT enable, as PathPlanner path generation causes Loop Time Spike
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Update field view with robot pose (real robot odometry; sim updates in simulationPeriodic)
    robotContainer.updateFieldPose();
    // robotContainer.republishDashboardChoosersOnNtConnect(); // TODO: Uncomment if Elastic SendableChooser is not working

    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();
    Logger.recordOutput("Dashboard/ShiftTimeRemaining", shiftInfo.remainingTime());
    Logger.recordOutput("Dashboard/ShootingActive", shiftInfo.active());
    Logger.recordOutput(
        "Dashboard/ShootingStatus", shiftInfo.active() ? "Our hub" : "Their hub");

    // Return to non-RT thread priority (do not modify the first argument)
    // Do NOT enable, as PathPlanner path generation causes Loop Time Spike
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      simMatchClockRunning = false;
      simDisableRobotWhenCountdownEnds = false;
      DriverStationSim.setMatchTime(-1);
      DriverStationSim.notifyNewData();
      SimMatchTimeCache.setRemainingSec(-1.0);
      TeamSignDisplayUtil.clearLatchedAutoHubScores();
    }
    robotContainer.idleAllSubsystems();
	}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      startSimMatchClock(20.0, true);
    }

    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    HubShiftUtil.initialize();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    if (Constants.currentMode == Constants.Mode.SIM) {
      startSimMatchClock(140.0, false);
      TeamSignDisplayUtil.latchAutoHubScoresFromFuelSim();
    }
    HubShiftUtil.initialize();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      DriverStationSim.setFmsAttached(true);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setMatchType(DriverStation.MatchType.Practice);
      DriverStationSim.setMatchTime(-1);
      SimMatchTimeCache.setRemainingSec(-1.0);
      // Desktop team-sign window: skip headless (e.g. some CI) where Swing would fail or hang.
      if (!GraphicsEnvironment.isHeadless()) {
        TeamSignSimWindow.open(
            TeamSignSimWindow.Role.BLUE_RP_FIELD,
            () -> TeamSignDisplayUtil.formatLineForSimulatedMatchBlue(0));
        TeamSignSimWindow.open(
            TeamSignSimWindow.Role.RED_RP_FIELD,
            () -> TeamSignDisplayUtil.formatLineForSimulatedMatchRed(0));
      }
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
    if (simMatchClockRunning) {
      double elapsed = Timer.getTimestamp() - simMatchStartTimestamp;
      double remaining = Math.max(0.0, simMatchDurationSec - elapsed);
      SimMatchTimeCache.setRemainingSec(remaining);
      DriverStationSim.setMatchTime(quantizeForFieldClockDisplay(remaining));
      // notifyNewData() is required: HAL_GetMatchTime reads a cache only refreshed here.
      DriverStationSim.notifyNewData();
      if (simDisableRobotWhenCountdownEnds && elapsed >= simMatchDurationSec) {
        simMatchClockRunning = false;
        simDisableRobotWhenCountdownEnds = false;
        CommandScheduler.getInstance().cancelAll();
        robotContainer.idleAllSubsystems();
        if (DriverStation.isAutonomousEnabled()) DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
      }
    }
  }

  /**
   * Starts the sim-only match countdown; remaining time is pushed in {@link #simulationPeriodic}. If
   * {@code disableWhenCountdownEnds}, the robot is disabled when time reaches zero (sim end of auto).
   */
  private void startSimMatchClock(double durationSec, boolean disableWhenCountdownEnds) {
    simMatchDurationSec = durationSec;
    simMatchStartTimestamp = Timer.getTimestamp();
    simMatchClockRunning = true;
    simDisableRobotWhenCountdownEnds = disableWhenCountdownEnds;
    SimMatchTimeCache.setRemainingSec(durationSec);
    DriverStationSim.setMatchTime(quantizeForFieldClockDisplay(durationSec));
    DriverStationSim.notifyNewData();
  }

  /**
   * Quantizes continuous remaining seconds into field-clock display seconds.
   *
   * <p>This prevents an early displayed {@code 0:00} during the final running second.
   */
  private static double quantizeForFieldClockDisplay(double remainingSec) {
    return Math.ceil(Math.max(0.0, remainingSec) - 1.0e-6);
  }
}
