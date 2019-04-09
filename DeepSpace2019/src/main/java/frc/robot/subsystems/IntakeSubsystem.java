
package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class IntakeSubsystem extends Subsystem {

  // Intake Talons for Cargo
  TalonSRX Intake = new TalonSRX(RobotMap.Intake);
  // Ground Hatch Intake/Outtake Talon
  TalonSRX GroundH = new TalonSRX(RobotMap.GroundH);
 
  public void initDefaultCommand() {
  }

  // This method takes in or shoots out a piece of cargo, depending on the direction of Talon spin given
  public void intake (boolean in, boolean out) {
    if (in) {
      // Set the 2 intake talons to rotate inwards (negative power at 50%)
      Intake.set(ControlMode.PercentOutput, -0.4);
    } else if (out) {
      Intake.set(ControlMode.PercentOutput, 0.55);
    } else {
      Intake.set(ControlMode.PercentOutput, 0);
    }
  }

  public void stopGroundH () {
    GroundH.set(ControlMode.PercentOutput, 0);
  }

}
