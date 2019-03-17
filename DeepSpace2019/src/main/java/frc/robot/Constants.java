
package frc.robot;

/**
 * Constant values for PID, Frame dimensions, Encoder ticks to angle/inches constants .etc
 */

public class Constants {

    // Drive Subsystem PID Constants
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static int kPIDLoopIdx = 0;

    // Drive PID minimal voltage constant
    public static double kMinimalVoltage = 0.25;
    // Autonomous Drive PID minimal voltage constant
    public static double kAutoMinimalVoltage = 0.13;

    // Tolerance of the PID for Gyro/Drive
    public static double kToleranceDegrees = 2.0;

    // PID timeout value
    public static int kTimeoutMs = 0;

    // Tolerance of the Arm PID (degrees)
    public static double kToleranceArm = 1.5;

    // Gyro Constants
    public static double kGyroRotationRate = 0.5;

    public static final double kMaxGyro = 0.5;

    public static boolean kCurrentLimited = false;

    // Drivetrain Encoder Values (inches to ticks)
    public static final double kDriveEncoder = 3.43774677003357 * 1024 / 12 / 2; //146.68
    // Arm Encoder Values (inches to ticks)
    public static final double kArmEncoder = 360.0/1024.0;

    // Arm PID Constants
    public static double kAP = 0.03; //0.05
    public static double kAI = 0.0;  //0.1
    public static double kAD = 0.0;  //0.3
    public static double kAF = 0.0;  //0.1

    // Vision Constants

    // Field view (FOV of camera)
    public static final double kVisionFieldViewWidthPx = 640;
    public static final double kVisionFieldViewHeightPx = 480;

    // Encoder positions (ticks) of each stage (256 tick encoder)
    public static final int kAEGH = 0;
    public static final int kAEL1 = -64;
    public static final int kAEL2 = -80;
    public static final int kAEDP = -140;

}
