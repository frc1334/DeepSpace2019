
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.sensors.LimitSwitch;

/**
 * Add your docs here.
 */

public class ArmSubsystem extends Subsystem {

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

  // Arm Talons

  // Arm base Talon
  TalonSRX ArmBase = new TalonSRX(RobotMap.ArmBase);

  // This double records the angle the arm is currently at
  public double angle;
  // This double records the destination angle (setpoint)
  public double dAngle;

  public void initDefaultCommand () {
    ArmBase.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

    ArmBase.config_kP(0, Constants.kAP);
    ArmBase.config_kI(0, Constants.kAI);
    ArmBase.config_kD(0, Constants.kAD);
    ArmBase.config_kF(0, Constants.kAF);

    ArmBase.configPeakOutputForward(1);
    ArmBase.configPeakOutputReverse(-1);
  }

  // This ethod updates the current angle
  public void updateAngle () {
    angle = ArmBase.getSelectedSensorPosition(0);
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

    updateAngle();

  }

  public void moveArmBasePercent (double power) {
    ArmBase.set(ControlMode.PercentOutput, (power * -0.75));
  }

  // This method sets the destination angle/set point
  public void setDestAngle (double dAngle) {
    this.dAngle = dAngle;
  }

  public void setPIDAngle (double setpoint) {
    ArmBase.set(ControlMode.Position, setpoint);
  }

  public double getCurrentAngle () {
    // Update the angle of the Arm
    updateAngle();

    return angle;
  }

  public boolean inRange (double setpoint) {
    if (Math.abs(setpoint - angle) <= Constants.kToleranceArm) {
      return true;
    }
    return false;
  }

}
