package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;

public class GyroCommand extends Command {

  double Angle;
  double Start, End;

  public GyroCommand () {
    requires(Robot.DriveSubsystem);
    this.Angle = 180;
  }
  
  public GyroCommand (double Angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.DriveSubsystem);
    this.Angle = Angle;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Start = System.currentTimeMillis();
    End = System.currentTimeMillis();
    Robot.DriveSubsystem.setSetpoint(Robot.DriveSubsystem.GyroDrive(Angle));
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    Robot.DriveSubsystem.usePIDOutput(Robot.DriveSubsystem.getPIDController().get());
    Robot.DriveSubsystem.ArcadeDrive(0, Constants.kGyroRotationRate);
    if(Math.abs(Robot.DriveSubsystem.getPIDController().getError()) > 2 ){
      Start = System.currentTimeMillis(); 
    } else {
      End = System.currentTimeMillis();
    }
  }

  // Make this return true when this Command no longer needs to run execute()s
  protected boolean isFinished() {
    if(End - Start > 200) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.DriveSubsystem.Left1.set(ControlMode.PercentOutput, 0);
    Robot.DriveSubsystem.Right1.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}