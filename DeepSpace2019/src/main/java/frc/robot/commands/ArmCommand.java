
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.OI;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {

  public ArmCommand () {
    requires(Robot.ArmSubsystem);
  }

  // Called just before this Command runs the first time
  protected void initialize () {
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute () {
    if (OI.positionArm0Deg()) {
      Robot.ArmSubsystem.dAngle = 0;
    } else if (OI.positionArm90Deg()) {
      Robot.ArmSubsystem.dAngle = 90;
    } else if (OI.positionArm135Deg()) {
      Robot.ArmSubsystem.dAngle = 135;
    }

    Robot.ArmSubsystem.setSetpoint(Robot.ArmSubsystem.dAngle);
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished () {
    return false;
  }

  // Called once after isFinished returns true
  protected void end () {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted () {
  }

}
