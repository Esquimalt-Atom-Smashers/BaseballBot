package frc.robot.simulation;

/**
 * Remaining match seconds for sim UI (e.g. team sign) updated from the robot/sim thread only.
 * Avoids reading {@link edu.wpi.first.wpilibj.DriverStation#getMatchTime()} on the Swing EDT, where
 * the DS packet cache can be stale or {@code -1} between updates.
 */
public final class SimMatchTimeCache {

    private static volatile double remainingSec = -1.0;

    private SimMatchTimeCache() {}

    /** Latest remaining seconds written from {@link frc.robot.Robot} sim match clock (or {@code -1}). */
    public static double getRemainingSec() {
        return remainingSec;
    } // End getRemainingSec

    /** Called from the robot loop when the sim match clock value changes. */
    public static void setRemainingSec(double seconds) {
        remainingSec = seconds;
    } // End setRemainingSec
}
