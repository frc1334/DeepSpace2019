
package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class WristSubsystem extends PIDSubsystem {

  public enum WristPos {
    DEFAULT,
    LOWGOAL,
    MEDGOAL,
    CARGO,
    INTAKE,
    CLIMB2,
    CLIMB3,
    PICKUP
  }
 
  // Wrist Talon
  TalonSRX Wrist = new TalonSRX(RobotMap.Wrist);

  // Wrist
  Potentiometer wPot = new AnalogPotentiometer(RobotMap.wPot, 360, 0);

  // This double records the angle the wrist is currently at
  public double angle;
  // This double records the destination angle (setpoint)
  public double dAngle;

  public WristSubsystem() {
    super("WristSubsystem", Constants.kWP, Constants.kWI, Constants.kWD);
    super.getPIDController().setInputRange(0.0, 360.0);
    super.getPIDController().setOutputRange(0.0, 360.0);
    super.getPIDController().setAbsoluteTolerance(Constants.kToleranceWrist);
    super.getPIDController().setContinuous(false);
  }

  public void initDefaultCommand() {
    Wrist.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

    Wrist.config_kP(0, Constants.kWP);
    Wrist.config_kI(0, Constants.kWI);
    Wrist.config_kD(0, Constants.kWD);
    Wrist.config_kF(0, Constants.kWF);

    Wrist.configPeakOutputForward(1);
    Wrist.configPeakOutputReverse(-1);
  }

  public void updateAngle () {
    // Update the current angle
    angle = Wrist.getSelectedSensorPosition(0);
  }

  public void setDAngle (double nDAngle) {
    // Update the destination angle (setpoint)
    dAngle = nDAngle;
  }

  public void moveWrist (boolean clockwise) {

    if (clockwise) {
      // If the limit switch does not hit anything and the arm is moving to release the limit switch
      Wrist.set(ControlMode.PercentOutput, 0.5);
    } else if (!clockwise) {
      // If the limit switch does not hit anything and the arm is moving to release the limit switch
      Wrist.set(ControlMode.PercentOutput, -0.5);
    }

    // Update the current angle
    angle = wPot.get();

  }

  protected double returnPIDInput() {
    updateAngle();

    return wPot.get();
  }

  public void setPIDAngle (double setpoint) {
    Wrist.set(ControlMode.Position, setpoint);
  } 

  protected void usePIDOutput(double output) {

    // Error term (destination angle - output, output is the current angle)
    double error = dAngle - output;

    // If the arm needs to move counter clockwise (the error is negative) - current position is behind destination
    if (Math.abs(error) > Constants.kToleranceArm && error < 0) {
      System.out.println("Moving Counter clockwise");
      moveWrist(true);
    } else if (Math.abs(error) > Constants.kToleranceArm && error > 0) {
      // If the arm needs to move clockwise (the error is positive) - current position is in front of destination
      moveWrist(false);
      System.out.println("Moving Clockwise");
    }

    // Update the angle of the Arm
    updateAngle();

  }

  public double getCurrentAngle () {
    updateAngle();
    return angle;
  }

  public boolean inRange (double setpoint) {
    if (Math.abs(setpoint - angle) <= Constants.kToleranceWrist) {
      return true;
    }
    return false;
  }

}
