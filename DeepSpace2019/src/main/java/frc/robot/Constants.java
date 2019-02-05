
package frc.robot;

/**
 * Constant values for PID, Frame dimensions, Encoder ticks to angle/inches constants .etc
 */

public class Constants {

    // Drive Subsystem PID Constants
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static int kPIDLoopIdx = 0;

    // Drive PID minimal voltage constant
    public static double kMinimalVoltage = 0.25;

    // Tolerance of the PID
    public static double kToleranceDegrees = 2.0;

    // Gyro Constants
    public static double kGyroRotationRate = 0.5;

    public static final double kMaxGyro = 0.5;

    public static boolean kCurrentLimited = false;

    // Drivetrain Encoder Values (inches to ticks)
    public static final double kDriveEncoder = 0.01;

    // Arm Encoder Values (inches to ticks)
    public static final double kArmEncoder = 0.01;

    // Arm PID Constants
    public static double kAP = 0.03;
    public static double kAI = 0.0;
    public static double kAD = 0.0;

}
