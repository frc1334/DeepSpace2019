
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

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

  // Intake Solenoids
  DoubleSolenoid IntakeSol = new DoubleSolenoid(RobotMap.IntakeSol1, RobotMap.IntakeSol2);

  // Arm Talons

  // Arm base Talon
  TalonSRX ArmBase = new TalonSRX(RobotMap.ArmBase);
  // Fore arm Talon
  TalonSRX ForeArm = new TalonSRX(RobotMap.ForeArm);

  // Limit Switch
  LimitSwitch ArmBound = new LimitSwitch(RobotMap.Switch);

  // This double records the angle the arm is currently at
  public double angle;
  // This double records the destination angle (setpoint)
  public double dAngle;

  public ArmSubsystem () {
    // Intert a subsystem name and PID values here
    super("ArmSubsystem", Constants.kAP, Constants.kAI, Constants.kAD);
    super.getPIDController().setInputRange(0.0, 180.0);
    super.getPIDController().setOutputRange(0.0, 180.0);
    super.getPIDController().setAbsoluteTolerance(Constants.kToleranceArm);
    super.getPIDController().setContinuous(true);
  }

  // This method takes in or shoots out a piece of cargo, depending on the direction of Talon spin given
  public void intake (boolean in, boolean out) {
    if (in && !out) {
      // Set the 2 intake talons to rotate inwards (negative power at 50%)
      Intake.set(ControlMode.PercentOutput, -1);
    } else if (out && !in) {
      Intake.set(ControlMode.PercentOutput, 1);
    } else {
      Intake.set(ControlMode.PercentOutput, 0);
    }
  }

  // This method is used to eject hatch panels off the intake
  public void hatchEject () {

    // Activate the solenoids
    IntakeSol.set(DoubleSolenoid.Value.kReverse);

    // Close solenoids after ejection
    IntakeSol.set(DoubleSolenoid.Value.kForward);

  }

  // This method moves the arm base talon (power also acts as direction, negative is counter clockwise and positive is clockwise)
  public void moveArmBase (boolean clockwise) {

    if (ArmBound.get()) {
      // Limit Switch activated - Reset encoder values
      angle = 0;
    }

    if (!ArmBound.get() && clockwise) {
      // If the limit switch does not hit anything and the arm is moving to release the limit switch
      ArmBase.set(ControlMode.PercentOutput, 0.6);
    } else if (!ArmBound.get() && !clockwise) {
      // If the limit switch does not hit anything and the arm is moving to release the limit switch
      ArmBase.set(ControlMode.PercentOutput, -0.6);
    } else if (ArmBound.get() && clockwise) {
      // The arm is hitting the limit switch and the arm is to be turnd clockwise (allow)
      ArmBase.set(ControlMode.PercentOutput, 0.6);
    }

    // Update the current angle
    angle = ArmBase.getSelectedSensorPosition(5) * Constants.kArmEncoder;

  }

  public void moveArmBasePercent (double power) {
    if (!ArmBound.get()) {
      ArmBase.set(ControlMode.PercentOutput, power);
    }
  }

  public void moveForeArmPercent (double power) {
    ForeArm.set(ControlMode.PercentOutput, power);

    // Update the current angle
    angle = ArmBase.getSelectedSensorPosition(5) * Constants.kArmEncoder;
  }

  // This method feeds a powerlevel of 0 into the arm (maintains level)
  public void maintainLevel () {
    ArmBase.set(ControlMode.PercentOutput, 0);
  }

  // This method moves the forearm of the arm (wrist)
  public void moveForeArm (boolean clockwise) {
    if (clockwise) {
      ForeArm.set(ControlMode.PercentOutput, 0.6);
    } else if (!clockwise) {
      ForeArm.set(ControlMode.PercentOutput, -0.6);
    }
  }

  // This method sets the destination angle/set point
  public void setDestAngle (double dAngle) {
    this.dAngle = dAngle;
  }

  public void initDefaultCommand () {
    ArmBase.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
  }

  protected double returnPIDInput () {
    // Return the current angle
    return ArmBase.getSelectedSensorPosition(5) * Constants.kArmEncoder;
  }

  protected void usePIDOutput (double output) {

    // Error term (destination angle - output, output is the current angle)
    double error = dAngle - output;

    // If the arm needs to move counter clockwise (the error is negative) - current position is behind destination
    if (Math.abs(error) > Constants.kToleranceArm && error < 0) {
      moveArmBase(false);
    } else if (Math.abs(error) > Constants.kToleranceArm && error > 0) {
      // If the arm needs to move clockwise (the error is positive) - current position is in front of destination
      moveArmBase(true);
    } else if (Math.abs(error) <= Constants.kToleranceArm) {
      // Otherwise, arm is in position (within a certain tolerance), maintain level
      maintainLevel();
    }

  }
}
