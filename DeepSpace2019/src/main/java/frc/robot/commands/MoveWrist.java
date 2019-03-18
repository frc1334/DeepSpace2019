/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class MoveWrist extends Command {

  double dAngle;

  WristSubsystem.WristPos position;

  public MoveWrist(double dAngle) {
    this.dAngle = dAngle;
    requires(Robot.WristSubsystem);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.WristSubsystem.setSetpoint(dAngle);
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
