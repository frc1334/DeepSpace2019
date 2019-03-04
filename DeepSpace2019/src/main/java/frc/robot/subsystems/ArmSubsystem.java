
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

  public enum Level {
    groundH,
    Lev1,
    Lev2,
    defaultPosition
  }
  
  // Intake Talons
  TalonSRX Intake = new TalonSRX(RobotMap.Intake);

  // Hatch Panel Clamp Intake
  TalonSRX Clamp = new TalonSRX(RobotMap.Clamp);

  // Arm Talons

  // Arm base Talon
  TalonSRX ArmBase = new TalonSRX(RobotMap.ArmBase);
  // Fore arm Talon
  TalonSRX ForeArm = new TalonSRX(RobotMap.ForeArm);

  // This double records the angle the arm is currently at
  public double angle;
  // This double records the destination angle (setpoint)
  public double dAngle;

  // This boolean is a toggle switch for the clamp direction (hold and release)
  public boolean toggleClamp = false;

  public ArmSubsystem () {
    // Intert a subsystem name and PID values here
    super("ArmSubsystem", Constants.kAP, Constants.kAI, Constants.kAD);
    super.getPIDController().setInputRange(0.0, 135.0);
    super.getPIDController().setOutputRange(0.0, 135.0);
    super.getPIDController().setAbsoluteTolerance(Constants.kToleranceArm);
    super.getPIDController().setContinuous(false);
  }

  // This ethod updates the curren angle
  public void updateAngle () {
    angle = ArmBase.getSelectedSensorPosition(0) * Constants.kArmEncoder;
  }

  // This method takes in or shoots out a piece of cargo, depending on the direction of Talon spin given
  public void intake (boolean in, boolean out) {
    if (in && !out) {
      // Set the 2 intake talons to rotate inwards (negative power at 50%)
      Intake.set(ControlMode.PercentOutput, -0.5);
    } else if (out && !in) {
      Intake.set(ControlMode.PercentOutput, 0.30);
    } else {
      Intake.set(ControlMode.PercentOutput, 0);
    }
  }

  // This method triggers whether to move, hold or release the clamp mechanism
  public void triggerClamp (boolean trigger) {
    // Default position is clamped
    if (trigger) {
      if (toggleClamp) {
        // Release the clamp
        for (int ticks=0; ticks<2000; ticks++) {
          Clamp.set(ControlMode.PercentOutput, 1);
        }
      } else if (!toggleClamp) {
        // Activate clamp
        for (int ticks=0; ticks<2000; ticks++) {
          Clamp.set(ControlMode.PercentOutput, -1);
        }
      }
      // Reverse the toggle switch
      toggleClamp = !toggleClamp;
    } else if (!trigger) {
      Clamp.set(ControlMode.PercentOutput, 0);
    }
  }

  // This method moves the arm base talon (power also acts as direction, negative is counter clockwise and positive is clockwise)
  public void moveArmBase (boolean clockwise) {

    if (clockwise) {
      // If the limit switch does not hit anything and the arm is moving to release the limit switch
      ArmBase.set(ControlMode.PercentOutput, 0.5);
    } else if (!clockwise) {
      // If the limit switch does not hit anything and the arm is moving to release the limit switch
      ArmBase.set(ControlMode.PercentOutput, -0.5);
    }

    // Update the current angle
    angle = ArmBase.getSelectedSensorPosition(0) * Constants.kArmEncoder;

    // Update the destination angle (setpoint)
    dAngle = ArmBase.getSelectedSensorPosition(0) * Constants.kArmEncoder;

  }

  public void moveArmBasePercent (double power) {
    ArmBase.set(ControlMode.PercentOutput, power);

    // Update the current angle
    dAngle = ArmBase.getSelectedSensorPosition(0) * Constants.kArmEncoder;
  }

  public void moveForeArmPercent (double power) {
    ForeArm.set(ControlMode.PercentOutput, power);

    // Update the current angle
    angle = ArmBase.getSelectedSensorPosition(0) * Constants.kArmEncoder;
  }

  // This method feeds a powerlevel of 0 into the arm (maintains level)
  public void maintainLevel () {
    ArmBase.set(ControlMode.PercentOutput, 0);
  }

  // This method moves the forearm of the arm (wrist)
  public void moveForeArm (boolean clockwise) {
    if (clockwise) {
      ForeArm.set(ControlMode.PercentOutput, 0.5);
    } else if (!clockwise) {
      ForeArm.set(ControlMode.PercentOutput, -0.5);
    }
  }

  // This method sets the destination angle/set point
  public void setDestAngle (double dAngle) {
    this.dAngle = dAngle;
  }

  public void initDefaultCommand () {
    ArmBase.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    ArmBase.setSelectedSensorPosition(0);
    ArmBase.configMotionAcceleration(25);
    ArmBase.configMotionCruiseVelocity(10);
  }

  protected double returnPIDInput () {
    // Return the current angle
    return ArmBase.getSelectedSensorPosition(0) * Constants.kArmEncoder;
  }

  protected void usePIDOutput (double output) {

    // Error term (destination angle - output, output is the current angle)
    double error = dAngle - output;

    // If the arm needs to move counter clockwise (the error is negative) - current position is behind destination
    if (Math.abs(error) > Constants.kToleranceArm && error < 0) {
      System.out.println("Moving Counter clockwise");
      moveArmBase(false);
    } else if (Math.abs(error) > Constants.kToleranceArm && error > 0) {
      // If the arm needs to move clockwise (the error is positive) - current position is in front of destination
      moveArmBase(true);
      System.out.println("Moving Clockwise");
    } else if (Math.abs(error) <= Constants.kToleranceArm) {
      // Otherwise, arm is in position (within a certain tolerance), maintain level
      maintainLevel();
      System.out.println("Within Range");
    }

  }

  public void setPosition (Level target) {
    // Move the talon to the matching encoder position based on each enum target case
    switch (target) {
      case groundH:
        setMotionMagicPosition(Constants.kAEGH);
        break;
      case Lev1:
        setMotionMagicPosition(Constants.kAEL1);
        break;
      case Lev2:
        setMotionMagicPosition(Constants.kAEL2);
        break;
      case defaultPosition:
        setMotionMagicPosition(Constants.kAEDP);
        break;
    }
  }

  public void setMotionMagicPosition(int encoderValue){
    ArmBase.set(ControlMode.MotionMagic, encoderValue);
  }

  public double getCurrentAngle () {
    return angle;
  }

}
