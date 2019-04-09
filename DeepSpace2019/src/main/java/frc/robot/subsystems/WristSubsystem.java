
package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class WristSubsystem extends Subsystem {

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

  // This double records the angle the wrist is currently at
  public double angle;
  // This double records the destination angle (setpoint)
  public double dAngle;

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

    updateAngle();

  }

  public void setPIDAngle (double setpoint) {
    Wrist.set(ControlMode.Position, setpoint);
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
