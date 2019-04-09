
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
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
    Robot.DriveSubsystem.initTalonFollower();
  }

  protected void execute() {

    // Climber code
    
    if (OI.climbDeploy()) {
      Robot.ClimberSubsystem.Climb(true);
    } else if (OI.unwind()) {
      Robot.ClimberSubsystem.Climb(false);
    } else {
      Robot.ClimberSubsystem.Stop();
    }

    // Drive Code

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
