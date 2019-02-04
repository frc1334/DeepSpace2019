
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoDriveCommand extends Command {

  double distance = 0;
  double error = 0;

  boolean inRange;

  // Start and end times
  long start, end;

  public AutoDriveCommand (double distance) {
    this.distance = distance;
    requires(Robot.DriveSubsystem);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    // Initialize start and end times
    start = System.currentTimeMillis();
    end = System.currentTimeMillis();
    // Initialize autonomous settings
    Robot.DriveSubsystem.initAutoCommand();
    
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

  // Called when another command which requires one or more of the same subsystems is scheduled to run
  protected void interrupted() {
  }
}
