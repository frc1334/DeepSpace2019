
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotMap;
import frc.robot.Constants;

/**
 * Add your docs here.
 */

public class DriveSubsystem extends PIDSubsystem {

  // Drivetrain TalonSRX map values (roboRIO port values) - 2 cim
  TalonSRX Left1 = new TalonSRX(RobotMap.Left1);
  VictorSPX Left2 = new VictorSPX(RobotMap.Left2);
  TalonSRX Right1 = new TalonSRX(RobotMap.Right1);
  VictorSPX Right2 = new VictorSPX(RobotMap.Right2);

  // AHRS - navX mxp - Gyro
  AHRS ahrs;

  // Gyro angle value
  double angle = 0;

  public DriveSubsystem () {
    // Calls parent constructor of PIDSubsystem with the parameters: "SubsystemName", kP, kI, kD
    super("DriveSubsystem", Constants.kP, Constants.kI, Constants.kD);
    super.getPIDController().setInputRange(-180.0,  180.0);
    super.getPIDController().setOutputRange(-1.0, 1.0);
    super.getPIDController().setAbsoluteTolerance(Constants.kToleranceDegrees);
    super.getPIDController().setContinuous(true);
  }

  public void initDefaultCommand () {

    // Initialize the Talons so that the Left2 and Right2 Talons will follow the movements of the Left1 and Right 1 Talons
    Left2.set(ControlMode.Follower, RobotMap.Left1);
    Right2.set(ControlMode.Follower, RobotMap.Right1);

    // Configure MagEncoders on the Left1 and Right1 Talons: args (FeedbackDevice, kPIDLoopIdx, TimeOutMS)
    Left1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, 50);
    Right1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, 50);

    // Set current limitting on the Left and Right 1 Talons to true
    Left1.enableCurrentLimit(true);
    Right1.enableCurrentLimit(true);

    // Initialize the AHRS sensor
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      resetGyroAngle();
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instancing navX MXP: " + ex.getMessage(), true);
    }
  
  }

  public void invertLeftTalons () {
    // Invert the Left Talons
    Left1.setInverted(false);
  }

  // Method to set the Drive Train to drive with PID
  public void setPIDDrive () {
    Left1.set(ControlMode.Position, 0);
    Right1.set(ControlMode.Position, 0);
  }

  public double GyroDrive (double turn) {
    angle += turn;
    int b = (int) angle/180;
    angle = (float) (angle * Math.pow(-1, b));
    return angle;
  }
  
  // Method to change gyro angle to 0 degrees
  public void resetGyroAngle () {
	  ahrs.reset();
	  angle = 0;
	}

  // Method to set the Drive Train to drive normally (without PID)
  public void setRegularDrive () {
    Left1.set(ControlMode.Velocity, 0);
    Right1.set(ControlMode.Velocity, 0);
  }

  // Basic Tank Drive Method
  public void TankDrive(double left, double right){
		Left1.set(ControlMode.PercentOutput, left);
		Right1.set(ControlMode.PercentOutput, right);
  }
  
  // Basic Arcade Drive Method
  public void ArcadeDrive (double speed, double turn) {
		TankDrive(-speed - turn, speed - turn);
  }

  // Method that "Soft-shifts" the Talons
  public void changeCurrentLimit () {

    if (Constants.kCurrentLimited) {
      Left1.configContinuousCurrentLimit(40);
      Right1.configContinuousCurrentLimit(40);

      Left1.configPeakCurrentLimit(40);
      Right1.configPeakCurrentLimit(40);
    } else {
      Left1.configContinuousCurrentLimit(60);
      Right1.configContinuousCurrentLimit(60);

      Left1.configPeakCurrentLimit(60);
      Right1.configPeakCurrentLimit(60);
    }

    Constants.kCurrentLimited = !Constants.kCurrentLimited;

  }

  // Method that returns the PID value of the gyro sensor
  protected double returnPIDInput () {
    return ahrs.pidGet();
  }

  // Method that adjusts the Gyro according to the PID outputs
  protected void usePIDOutput (double output) {
    if (output > Constants.kMaxGyro) {
      Constants.kGyroRotationRate = Constants.kMaxGyro;
    } else if (output <= -Constants.kMaxGyro) {
      Constants.kGyroRotationRate = -Constants.kMaxGyro;
    } else {
      // In between maximum and minimum
      
    }
  }

}
