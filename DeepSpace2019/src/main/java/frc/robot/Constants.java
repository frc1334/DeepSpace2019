
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
    public static double kToleranceArm = 5;
    public static double kToleranceWrist = 5;

    // Gyro Constants
    public static double kGyroRotationRate = 0.5;

    public static final double kMaxGyro = 0.5;

    public static boolean kCurrentLimited = false;

    // Drivetrain Encoder Values (inches to ticks)
    public static final double kDriveEncoder = 3.43774677003357 * 1024 / 12 / 2; //146.68
    // Arm Encoder Values (inches to ticks)
    public static final double kArmEncoder = 360.0/1024.0;

    // Arm PID Constants
    public static double kAP = 0.1; //0.05
    public static double kAI = 0.1;  //0.1
    public static double kAD = 0.1;  //0.3
    public static double kAF = 0.1;  //0.1

    // Wrist PID Constants
    public static double kWP = 0.03;
    public static double kWI = 0.0;
    public static double kWD = 0.0;
    public static double kWF = 0.0;

    // Arm Positions
    public static double aDEFAULT = 0.0;
    public static double aCARGO = 0.0;
    public static double aLEV1ROCKET = 0.0;
    public static double aINTAKE = 0.0;
    public static double aCLIMB2 = 0.0;
    public static double aCLIMB3 = 0.0;
    public static double aPICKUP = 0.0;

    // Wrist Positions
    public static double wDEFAULT = 0.0;
    public static double wCARGO = 0.0;
    public static double wLEV1ROCKET = 0.0;
    public static double wINTAKE = 0.0;
    public static double wCLIMB2 = 0.0;
    public static double wCLIMB3 = 0.0;
    public static double wPICKUP = 0.0;

}
