
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.sensors.LimitSwitch;

/**
 * Add your docs here.
 */

public class ArmSubsystem extends PIDSubsystem {
  
  // Intake Talons
  TalonSRX Intake = new TalonSRX(RobotMap.Intake);

  // Arm Talons

  // Arm base Talon
  TalonSRX ArmBase = new TalonSRX(RobotMap.ArmBase);
  // Fore arm Talon
  TalonSRX ForeArm = new TalonSRX(RobotMap.ForeArm);

  // Limit Switch
  LimitSwitch ArmBound = new LimitSwitch(RobotMap.Switch);

  // This double records the angle the arm is currently at
  double angle;
  // This double records the destination angle (setpoint)
  double dAngle;

  public ArmSubsystem () {
    // Intert a subsystem name and PID values here
    super("ArmSubsystem", Constants.kAP, Constants.kAI, Constants.kAD);
  }

  // This method takes in or shoots out a piece of cargo, depending on the direction of Talon spin given
  public void intake (boolean in, boolean out) {
    if (in && !out) {
      // Set the 2 intake talons to rotate inwards (negative power at 50%)
      Intake.set(ControlMode.PercentOutput, -0.5);
    } else if (out && !in) {
      Intake.set(ControlMode.PercentOutput, 1);
    } else {
      Intake.set(ControlMode.PercentOutput, 0);
    }
  }

  // This method moves the arm base talon (power also acts as direction, negative is counter clockwise and positive is clockwise)
  public void moveArmBase (double power) {

    if (ArmBound.get()) {
      // Limit Switch activated - Reset encoder values
      ArmBase.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
    }

    if (!ArmBound.get() && power > 0) {
      // If the limit switch does not hit anything and the arm is moving to release the limit switch
      ArmBase.set(ControlMode.PercentOutput, power);
      // Update the current angle
      angle = ArmBase.getSelectedSensorPosition(5) * Constants.kArmEncoder;
    }

  }

  // This method moves the forearm of the arm
  public void moveForeArm (double power) {
    ForeArm.set(ControlMode.PercentOutput, power);
  }

  // This method sets the destination angle/set point
  public void setDestAngle (double dAngle) {
    this.dAngle = dAngle;
  }

  // This method moves the arm base to a certain degree position (from 0 - 180)
  public void moveArmBaseToDegree (double destDegree) {
    // How many degrees the arm base needs to move
    double mValue = destDegree - angle;
  }

  // This method moves the fore arm to a certain degree position (from 0 - 180)
  public void moveForeArmToDegree (double destDegree) {
    // How many degrees the fore arm needs to move
    double mValue = destDegree - angle;
  }

  public void initDefaultCommand () {
    ArmBase.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
  }

  protected double returnPIDInput () {
    // Return the current angle that the arm is at
    return ArmBase.getSelectedSensorPosition(5) * Constants.kArmEncoder;
  }

  protected void usePIDOutput (double output) {

    // Error term (destination angle - output, output is the current angle)
    double error = dAngle - output;

    // If the arm needs to move counter clockwise (the error is negative) - current position is behind destination
    if (error < 0) {
      moveArmBase(-0.5);
    } else if (error > 0) {
      // If the arm needs to move clockwise (the error is positive) - current position is in front of destination
      moveArmBase(0.5);
    }

  }
}
