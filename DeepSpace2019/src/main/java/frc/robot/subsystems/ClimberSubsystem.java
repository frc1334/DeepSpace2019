
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.RobotMap;

public class ClimberSubsystem extends Subsystem {

  // Climber Talon
  TalonSRX Climb = new TalonSRX(RobotMap.Climber);
  
  public void initDefaultCommand() {
    
  }

  public void ActivateClimbTalon () {
    Climb.set(ControlMode.PercentOutput, 1);
  }

}
