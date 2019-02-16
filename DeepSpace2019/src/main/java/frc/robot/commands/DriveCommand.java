
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.OI;

public class DriveCommand extends Command {
  
  public DriveCommand() {
    requires(Robot.DriveSubsystem);
    requires(Robot.ClimberSubsystem);
  }

  protected void initialize() {
    Robot.DriveSubsystem.invertLeftTalons();
  }

  protected void execute() {

    // Climber deploy
    if (OI.deployClimber()) {

    }

    // Clamp the climber onto the hab station if neccesary
    Robot.ClimberSubsystem.Clamp(OI.climberClamp());

    // Backflip with the climber if neccesary (hold button)
    Robot.ClimberSubsystem.Climb(OI.climberBackFlip());

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
