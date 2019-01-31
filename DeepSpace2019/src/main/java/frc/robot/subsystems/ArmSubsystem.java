
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.RobotMap;
import frc.robot.sensors.LimitSwitch;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends PIDSubsystem {
  
  // Intake Talons
  TalonSRX Intake1 = new TalonSRX(RobotMap.Intake1);
  TalonSRX Intake2 = new TalonSRX(RobotMap.Intake2);

  // Arm Talons

  // Arm base Talon
  TalonSRX ArmBase = new TalonSRX(RobotMap.ArmBase);
  // Fore arm Talon
  TalonSRX ForeArm = new TalonSRX(RobotMap.ForeArm);

  // Limit Switch
  LimitSwitch top = new LimitSwitch(RobotMap.Switch);

  // This double records the angle the arm is currently at
  double angle;

  public ArmSubsystem() {
    // Intert a subsystem name and PID values here
    super("ArmSubsystem", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  // This method takes in or shoots out a piece of cargo, depending on the direction of Talon spin given
  public void intake (boolean in, boolean out) {
    if (in && !out) {
      // Set the 2 intake talons to rotate inwards (negative power at 50%)
      Intake1.set(ControlMode.PercentOutput, -0.5);
      Intake2.set(ControlMode.PercentOutput, -0.5);
    } else if (out && !in) {
      Intake1.set(ControlMode.PercentOutput, 1);
      Intake2.set(ControlMode.PercentOutput, 1);
    } else {
      Intake1.set(ControlMode.PercentOutput, 0);
      Intake2.set(ControlMode.PercentOutput, 0);
    }
  }

  // This method moves the arm base talon
  public void moveArmBase (double power) {
    ArmBase.set(ControlMode.PercentOutput, power);
  }

  // This method moves the forearm of the arm
  public void moveForeArm (double power) {
    ForeArm.set(ControlMode.PercentOutput, power);
  }

  // This method moces the arm base by n degrees
  public void moveArmBaseDegrees (double degrees) {

  }

  // This method moces the forearm by n degrees
  public void moveForeArmDegrees (double degrees) {
    
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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
}
