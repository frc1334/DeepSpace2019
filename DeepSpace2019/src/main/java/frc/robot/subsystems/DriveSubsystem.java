
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

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

  public DriveSubsystem() {
    // Calls parent constructor of PIDSubsystem with the parameters: "SubsystemName", kP, kI, kD
    super("DriveSubsystem", Constants.kP, Constants.kI, Constants.kD);
  }

  public void initDefaultCommand() {
    // Initialize the Talons so that the Left2 and Right2 Talons will follow the movements of the Left1 and Right 1 Talons
    Left2.set(ControlMode.Follower, RobotMap.Left1);
    Right2.set(ControlMode.Follower, RobotMap.Right1);
  }

  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return 0.0;
  }

  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
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
