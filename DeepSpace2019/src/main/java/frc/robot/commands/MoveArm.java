
package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class MoveArm extends Command {

  double dAngle;

  public MoveArm(double dAngle) {
    requires(Robot.ArmSubsystem);

    this.dAngle = dAngle;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    System.out.println("MOVING ARM ----------------------------");
    Robot.ArmSubsystem.setPIDAngle(dAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    if (Robot.ArmSubsystem.inRange(dAngle)) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }

}
