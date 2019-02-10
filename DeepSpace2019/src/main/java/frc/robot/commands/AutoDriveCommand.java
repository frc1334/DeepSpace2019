
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Robot;
import frc.robot.Constants;

public class AutoDriveCommand extends Command {

  double distance = 0;
  double error = 0;
  double ticks = 0;

  boolean inRange = false;

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
    
    int LeftAbsolutePosition = Robot.DriveSubsystem.Left1.getSensorCollection().getPulseWidthPosition();
		int RightAbsolutePosition = Robot.DriveSubsystem.Right1.getSensorCollection().getPulseWidthPosition();

    // Mask overflows, keep values under 12 bits
    LeftAbsolutePosition &= 0xFFF;
    RightAbsolutePosition &= 0xFFF;

    Robot.DriveSubsystem.setQuadAbsolute(LeftAbsolutePosition, RightAbsolutePosition);

    ticks = distance * Constants.kDriveEncoder;

    Robot.DriveSubsystem.Left1.set(ControlMode.MotionMagic, ticks);
    Robot.DriveSubsystem.Right1.set(ControlMode.MotionMagic, ticks);
    Robot.DriveSubsystem.Left1.config_IntegralZone(Constants.kPIDLoopIdx, 2, Constants.kTimeoutMs);
    Robot.DriveSubsystem.Right1.config_IntegralZone(Constants.kPIDLoopIdx, 2, Constants.kTimeoutMs);

  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    // Get the error of the left drive encoder
    error = ticks - Robot.DriveSubsystem.Left1.getSelectedSensorPosition(0);

    if(Math.abs(error)<=300 && end - start < -500){ //DO NOT REMOVE THE TIME REQUIREMENT OR ELSE THE PREVIOUS ERROR WILL REMAIN AND THE COMMAND WILL AUTOMATICALLY START MOVING SLOWLY
      Robot.DriveSubsystem.isClose = true;
    }

    if (Robot.DriveSubsystem.isClose) {
      if (Math.signum(error) == -1 && Math.abs(error)>=100) {
        Robot.DriveSubsystem.Left1.set(ControlMode.PercentOutput, -Constants.kAutoMinimalVoltage);
        Robot.DriveSubsystem.Right1.set(ControlMode.PercentOutput, -Constants.kAutoMinimalVoltage);
      } else if(Math.signum(error) == 1 && Math.abs(error)>=100) {
        Robot.DriveSubsystem.Left1.set(ControlMode.PercentOutput, Constants.kAutoMinimalVoltage);
        Robot.DriveSubsystem.Right1.set(ControlMode.PercentOutput, Constants.kAutoMinimalVoltage);
      } else {
        Robot.DriveSubsystem.Left1.set(ControlMode.PercentOutput, 0);
        Robot.DriveSubsystem.Right1.set(ControlMode.PercentOutput, 0);
      }
    }

    inRange = Math.abs(ticks-Robot.DriveSubsystem.Left1.getSelectedSensorPosition(0) ) <= 100;
    if (inRange) {
      end = System.currentTimeMillis();
    }
    else {
      start = System.currentTimeMillis();
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    if (end - start > 200) {
        return true; 
    }
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    Robot.DriveSubsystem.isClose = false;
    Robot.DriveSubsystem.Left1.set(ControlMode.PercentOutput, 0);
    Robot.DriveSubsystem.Right1.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same subsystems is scheduled to run
  protected void interrupted() {
  }
}
