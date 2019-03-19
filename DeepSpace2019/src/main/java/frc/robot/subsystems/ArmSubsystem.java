
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
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

  public enum ArmPos {
    DEFAULT,
    LOWGOAL,
    MEDGOAL,
    CARGO,
    INTAKE,
    CLIMB2,
    CLIMB3,
    PICKUP
  }
  
  // Intake Talons for Cargo
  TalonSRX Intake = new TalonSRX(RobotMap.Intake);
  // Ground Hatch Intake/Outtake Talon
  TalonSRX GroundH = new TalonSRX(RobotMap.GroundH);

  // Arm Talons

  // Arm base Talon
  TalonSRX ArmBase = new TalonSRX(RobotMap.ArmBase);

  // This double records the angle the arm is currently at
  public double angle;
  // This double records the destination angle (setpoint)
  public double dAngle;

  Potentiometer aPot = new AnalogPotentiometer(0);

  public ArmSubsystem () {
    // Intert a subsystem name and PID values here
    super("ArmSubsystem", Constants.kAP, Constants.kAI, Constants.kAD);
    super.getPIDController().setInputRange(0.0, 360.0);
    super.getPIDController().setOutputRange(0.0, 360.0);
    super.getPIDController().setAbsoluteTolerance(Constants.kToleranceArm);
    super.getPIDController().setContinuous(false);
  }

  // This ethod updates the current angle
  public void updateAngle () {
    angle = aPot.get();
  }

  // This method updates the destination angle (for manual control and staying in place)
  public void updateDAngle () {
    dAngle = aPot.get();
  }

  // This method takes in or shoots out a piece of cargo, depending on the direction of Talon spin given
  public void intake (boolean in, boolean out) {
    if (in && !out) {
      // Set the 2 intake talons to rotate inwards (negative power at 50%)
      Intake.set(ControlMode.PercentOutput, -0.40);
    } else if (out && !in) {
      Intake.set(ControlMode.PercentOutput, 0.40);
    } else {
      Intake.set(ControlMode.PercentOutput, 0);
    }
  }

  // This method is used to toggle and activate the ground hatch pickup talon
  public void groundToggle (boolean toggleGroundHatch) {
    if (toggleGroundHatch) {
      GroundH.set(ControlMode.PercentOutput, 0.40);
    } else if (!toggleGroundHatch) {
      GroundH.set(ControlMode.PercentOutput, -0.40);
    }
  }

  public void stopGroundH () {
    GroundH.set(ControlMode.PercentOutput, 0);
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
    angle =  aPot.get();

    // Update the destination angle (setpoint)
    dAngle =  aPot.get();

  }

  public void moveArmBasePercent (double power) {
    ArmBase.set(ControlMode.PercentOutput, (power * -0.75));

    // Update the current angle
    dAngle =  aPot.get();
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
    return aPot.get();
  }

  protected void usePIDOutput (double output) {

    // Error term (destination angle - output, output is the current angle)
    double error = dAngle - output;

    // If the arm needs to move counter clockwise (the error is negative) - current position is behind destination
    if (Math.abs(error) > Constants.kToleranceArm && error < 0) {
      System.out.println("Moving Counter clockwise");
      moveArmBase(true);
    } else if (Math.abs(error) > Constants.kToleranceArm && error > 0) {
      // If the arm needs to move clockwise (the error is positive) - current position is in front of destination
      moveArmBase(false);
      System.out.println("Moving Clockwise");
    }

    // Update the angle of the Arm
    updateAngle();

  }

  public double getCurrentAngle () {
    return angle;
  }

  public boolean inRange (double setpoint) {
    if (Math.abs(setpoint - angle) <= Constants.kToleranceArm) {
      return true;
    }
    return false;
  }

}
