
package frc.robot;

/**
 * Constant values for PID, Frame dimensions, Field Dimensions .etc
 */

public class Constants {

    // Drive Subsystem PID Constants
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static int kPIDLoopIdx = 0;

    // Tolerance of the PID
    public static double kToleranceDegrees = 2.0;

    // Gyro Constants
    public static double kGyroRotationRate = 0.5;

    public static final double kMaxGyro = 0.5;

    public static boolean kCurrentLimited = false;

}
