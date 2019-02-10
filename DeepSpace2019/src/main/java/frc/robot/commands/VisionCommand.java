
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.Constants;

public class VisionCommand extends Command {

  public VisionCommand() {
    requires(Robot.DriveSubsystem);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
  }

  // This method is used for the widgets on the left side of the camera
  public boolean getLeftVisionInView (double x, double y) {

    // This variable stores the center x values (average of the 2 x coordinates)
    double centerPointX = (Constants.x1 + Constants.x2) / 2;
    // This variable stores the center y values (average of the 2 y coordinates)
    double centerPointY = (Constants.y1 + Constants.y2) / 2;

    // If the target is approximately 
    if (Math.abs(x - centerPointX) >= 270) {
      return true;
    }

    return false;
  }

  // This method is used for the widgets on the right side of the camera
  public boolean getRightVisionInView (double x, double y) {

    // This variable stores the center x values (average of the 2 x coordinates)
    double centerPointX = (Constants.x1 + Constants.x2) / 2;
    // This variable stores the center y values (average of the 2 y coordinates)
    double centerPointY = (Constants.y1 + Constants.y2) / 2;

    if (Math.abs(x - centerPointX) <= 370) {
      return true;
    }

    return false;
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
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
