
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.OI;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {

  public ArmCommand () {
    requires(Robot.ArmSubsystem);
  }

  // Called just before this Command runs the first time
  protected void initialize () {
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute () {

    // Check OI to see if the destination angle (setpoint) of the arm PID needs to be updated

    if (OI.positionArm0Deg()) {
      Robot.ArmSubsystem.dAngle = 0;
    } else if (OI.positionArm90Deg()) {
      Robot.ArmSubsystem.dAngle = 90;
    } else if (OI.positionArm135Deg()) {
      Robot.ArmSubsystem.dAngle = 135;
    }

    if (OI.getArmBasePercent() >= 0.1) {
      // Move the arm via variable control
      Robot.ArmSubsystem.moveArmBasePercent(OI.getArmBasePercent());
    } else {
      Robot.ArmSubsystem.maintainLevel();
    }

    // Move the wrist via variable control
    Robot.ArmSubsystem.moveForeArmPercent(OI.getForeArmPercent());

    // Use intake
    Robot.ArmSubsystem.intake(OI.getIntake(), OI.getOuttake());

    // Move the arm via PID to the setpoint (either maintain level or continue course if the value was not changed or move to new angle)
    Robot.ArmSubsystem.setSetpoint(Robot.ArmSubsystem.dAngle);

  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished () {
    // if (Robot.ArmSubsystem.angle == Robot.ArmSubsystem.dAngle) {
    //   return true;
    // }
    return false;
  }

  // Called once after isFinished returns true
  protected void end () {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted () {
  }

}
