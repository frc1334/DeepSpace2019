
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.RobotMap;
import frc.robot.Constants;

public class ArmSubsystemEncoder extends Subsystem {

  TalonSRX Arm = new TalonSRX(RobotMap.ArmBase);

  public enum Level {
    groundH,
    Lev1,
    Lev2,
    defaultPosition
  }

  public void initDefaultCommand () {
    Arm.config_kP(0, Constants.kAP);
    Arm.config_kI(0, Constants.kAI);
    Arm.config_kD(0, Constants.kAD);
    Arm.config_kF(0, Constants.kAF);
    Arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    Arm.setSelectedSensorPosition(0);
    Arm.configMotionAcceleration(60);
    Arm.configMotionCruiseVelocity(200);
  }

  public void setPosition (Level target) {
    switch (target) {
      case groundH:
        break;
      case Lev1:
        break;
      case Lev2:
        break;
      case defaultPosition:
        break;
    }
  }

  public void setMotionMagicPosition(int encoderValue){
    Arm.set(ControlMode.MotionMagic, encoderValue);
  }

}
