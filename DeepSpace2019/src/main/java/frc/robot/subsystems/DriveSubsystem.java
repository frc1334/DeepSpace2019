
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
  TalonSRX Left2 = new TalonSRX(RobotMap.Left2);
  TalonSRX Right1 = new TalonSRX(RobotMap.Right1);
  TalonSRX Right2 = new TalonSRX(RobotMap.Right2);

  // AHRS - navX mxp - Gyro
  AHRS ahrs;

  // Gyro angle
  double angle = 0;

  public DriveSubsystem() {
    // Calls parent constructor of PIDSubsystem with the parameters: "SubsystemName", kP, kI, kD
    super("DriveSubsystem", Constants.kP, Constants.kI, Constants.kD);
  }

  public void initDefaultCommand() {

    // Initialize the Talons so that the Left2 and Right2 Talons will follow the movements of the Left1 and Right 1 Talons
    Left2.set(ControlMode.Follower, RobotMap.Left1);
    Right2.set(ControlMode.Follower, RobotMap.Right1);

    // Configure MagEncoders on the Left1 and Right1 Talons: args (FeedbackDevice, kPIDLoopIdx, TimeOutMS)
    Left1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, 50);
    Right1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, 50);

    // Initialize the AHRS sensor
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      ResetGyroAngle();
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instancing navX MXP: " + ex.getMessage(), true);
    }
  
  }

  // Method that returns the PID value of the gyro sensor
  protected double returnPIDInput() {
    return ahrs.pidGet();
  }

  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
  }

  // Method to set the Drive Train to drive with PID
  public void setPIDDrive () {
    Left1.set(ControlMode.Position, 0);
    Right1.set(ControlMode.Position, 0);
  }

  // Method to change gyro angle to 0 degrees
  public void ResetGyroAngle(){
	  ahrs.reset();
	  angle = 0;
	}

  // Method to set the Drive Train to drive normally (without PID)
  public void setRegularDrive () {
    Left1.set(ControlMode.Velocity, 0);
    Right1.set(ControlMode.Position, 0);
  }

  // Basic Tank Drive Method
  public void TankDrive(double left, double right){
		Left1.set(ControlMode.PercentOutput, left);
		Left2.set(ControlMode.PercentOutput, left);
		Right1.set(ControlMode.PercentOutput, right);
		Right2.set(ControlMode.PercentOutput, right);
  }
  
  // Basic Arcade Drive Method
  public void ArcadeDrive(double speed, double turn){
		TankDrive(-speed -turn, speed - turn);
	}

}
