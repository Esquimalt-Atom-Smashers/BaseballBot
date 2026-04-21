// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.kActivePreshootTime;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;

public class HubShiftUtil {
    public enum ShiftEnum {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED;
    }

    public record ShiftInfo(ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

    private static Timer shiftTimer = new Timer();
    private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

    private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static final double approachingActiveFudge = -kActivePreshootTime.in(Seconds);

    public static final double autoEndTime = 20.0;
    public static final double teleopDuration = 140.0;
    private static final boolean[] activeSchedule = {true, true, false, true, false, true};
    private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

    private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

    public static Optional<Boolean> getAllianceWinOverride() {
        return allianceWinOverride.get();
    }

    public static void setAllianceWinOverride(Supplier<Optional<Boolean>> allianceWinOverride) {
        HubShiftUtil.allianceWinOverride = allianceWinOverride;
    }

    public static Alliance getFirstActiveAlliance() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Return override value
        var winOverride = getAllianceWinOverride();
        if (!winOverride.isEmpty()) {
            return winOverride.get()
                    ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
                    : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
        }

        // Return FMS value
        String message = DriverStation.getGameSpecificMessage();
        if (message.length() > 0) {
            char character = message.charAt(0);
            if (character == 'R') {
                return Alliance.Blue;
            } else if (character == 'B') {
                return Alliance.Red;
            }
        }

        // Return default value
        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    /** Starts the timer at the begining of teleop. */
    public static void initialize() {
        shiftTimer.restart();
    }

    /**
     * Selects the active vs inactive shift schedule seen by the current driver station alliance.
     *
     * @param flipFromDs when true, returns the opposite alliance's schedule
     */
    private static boolean[] scheduleForDsAlliance(boolean flipFromDs) {
        Alliance startAlliance = getFirstActiveAlliance();
        boolean activeFirst = startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue);
        if (flipFromDs) {
            activeFirst = !activeFirst;
        }
        return activeFirst ? activeSchedule : inactiveSchedule;
    }

    private static boolean[] getSchedule(boolean override) {
        return scheduleForDsAlliance(override);
    }

    private static boolean[] getSchedule() {
        return getSchedule(false);
    }

    private static ShiftInfo getShiftInfo(boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
        double currentTime = shiftTimer.get();
        double stateTimeElapsed = shiftTimer.get();
        double stateTimeRemaining = 0.0;
        boolean active = false;
        ShiftEnum currentShift = ShiftEnum.DISABLED;

        if (DriverStation.isAutonomousEnabled()) {
            stateTimeElapsed = currentTime;
            stateTimeRemaining = autoEndTime - currentTime;
            active = true;
            currentShift = ShiftEnum.AUTO;
        } else if (DriverStation.isEnabled()) {
            int currentShiftIndex = -1;
            for (int i = 0; i < shiftStartTimes.length; i++) {
                if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }
            if (currentShiftIndex < 0) {
                // After last shift, so assume endgame
                currentShiftIndex = shiftStartTimes.length - 1;
            }

            // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
            stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
            stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

            // If the state is the same as the last shift, combine the elapsed time
            if (currentShiftIndex > 0) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
                    stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
                }
            }

            // If the state is the same as the next shift, combine the remaining time
            if (currentShiftIndex < shiftEndTimes.length - 1) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
                    stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
                }
            }

            active = currentSchedule[currentShiftIndex];
            currentShift = shiftsEnums[currentShiftIndex];
        }
        ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
        return shiftInfo;
    }

    public static ShiftInfo getOfficialShiftInfo() {
        return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
    }

    /**
     * Returns shift timing and {@link ShiftInfo#active()} hub-scoring flag for the given alliance, using the same
     * teleop time windows as {@link #getOfficialShiftInfo()} but selecting the active vs inactive band when the
     * alliance differs from the current driver station alliance.
     *
     * @param redAlliance {@code true} for the red alliance hub schedule, {@code false} for blue
     * @return shift index, elapsed/remaining times in the current window, and whether that alliance may score
     */
    public static ShiftInfo getOfficialShiftInfoForAlliance(boolean redAlliance) {
        boolean dsIsRed = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
        boolean flipFromDs = redAlliance != dsIsRed;
        return getShiftInfo(scheduleForDsAlliance(flipFromDs), shiftStartTimes, shiftEndTimes);
    }

    public static ShiftInfo getShiftedShiftInfo(boolean override) {
        boolean[] shiftSchedule = getSchedule(override);
        // Starting active
        if (shiftSchedule[1] == true) {
            double[] shiftedShiftStartTimes = {
                0.0, 10.0, 35.0, 60.0 + approachingActiveFudge, 85.0, 110.0 + approachingActiveFudge
            };
            double[] shiftedShiftEndTimes = {
                10.0, 35.0, 60.0 + approachingActiveFudge, 85.0, 110.0 + approachingActiveFudge, 140.0
            };
            return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
        }
        double[] shiftedShiftStartTimes = {
            0.0, 10.0, 35.0 + approachingActiveFudge, 60.0, 85.0 + approachingActiveFudge, 110.0
        };
        double[] shiftedShiftEndTimes = {
            10.0, 35.0 + approachingActiveFudge, 60.0, 85.0 + approachingActiveFudge, 110.0, 140.0
        };
        return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }

    public static ShiftInfo getShiftedShiftInfo() {
        return getShiftedShiftInfo(false);
    }
}
