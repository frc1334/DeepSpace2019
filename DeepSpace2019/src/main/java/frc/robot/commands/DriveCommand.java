
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.OI;

public class DriveCommand extends Command {
  
  public DriveCommand() {
    requires(Robot.DriveSubsystem);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.DriveSubsystem.invertLeftTalons();
  }

  protected void execute() {
    // Drive based on the response of the Driver's speed and steer
    Robot.DriveSubsystem.ArcadeDrive(OI.getDriverSpeed(), OI.getDriverSteer());
    // Change the Current Limit on the Drivetrain's Talons if neccesary
    if (OI.changeDriverCurrentLimit()) {
      Robot.DriveSubsystem.changeCurrentLimit();
    }
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
  }

  protected void interrupted() {
  }
}
