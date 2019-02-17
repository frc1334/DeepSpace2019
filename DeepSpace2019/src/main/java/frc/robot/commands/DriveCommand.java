
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.OI;

/*

  Drive Command is the command class that controls both the
  Drivetrain (DriveSubsystem) and the Climber (ClimberSubsystem)

*/

public class DriveCommand extends Command {
  
  public DriveCommand() {
    requires(Robot.DriveSubsystem);
    requires(Robot.ClimberSubsystem);
  }

  protected void initialize() {
    Robot.DriveSubsystem.invertLeftTalons();
  }

  protected void execute() {

    // Climber code
    
    if (!OI.climberBackFlip() && !OI.climberFrontFlip()) {
      Robot.ClimberSubsystem.Stop();
    } else if (OI.climberBackFlip()) {
      // Backflip with the climber if neccesary (hold button)
      Robot.ClimberSubsystem.Climb(false);
    } else if (OI.climberFrontFlip()) {
      Robot.ClimberSubsystem.Climb(true);
    }

    // Drive Code

    // Change the Current Limit on the Drivetrain's Talons if neccesary
    if (OI.changeDriverCurrentLimit()) {
      Robot.DriveSubsystem.changeCurrentLimit();
    }

    // Drive based on the response of the Driver's speed and steer
    Robot.DriveSubsystem.ArcadeDrive(OI.getDriverSpeed(), OI.getDriverSteer());

  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
  }

  protected void interrupted() {
  }

}
