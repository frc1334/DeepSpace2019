
package frc.robot.commands;

import java.util.function.ObjIntConsumer;

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

    // Update the current angle position
    Robot.ArmSubsystem.updateAngle();

    Robot.ArmSubsystem.dAngle = Robot.ArmSubsystem.angle;

    // Check OI to see if the destination angle (setpoint) of the arm PID needs to be updated

    if (OI.positionArm0Deg()) {
      Robot.ArmSubsystem.dAngle = 0;
      // Move the arm via PID to the setpoint (either maintain level or continue course if the value was not changed or move to new angle)
      Robot.ArmSubsystem.setSetpoint(Robot.ArmSubsystem.dAngle);
    } else if (OI.positionArm90Deg()) {
      Robot.ArmSubsystem.dAngle = 90;
      // Move the arm via PID to the setpoint (either maintain level or continue course if the value was not changed or move to new angle)
      Robot.ArmSubsystem.setSetpoint(Robot.ArmSubsystem.dAngle);
    } else if (OI.positionArm135Deg()) {
      Robot.ArmSubsystem.dAngle = 135;
      // Move the arm via PID to the setpoint (either maintain level or continue course if the value was not changed or move to new angle)
      Robot.ArmSubsystem.setSetpoint(Robot.ArmSubsystem.dAngle);
    }

    // if (Math.abs(OI.getArmBasePercent()) > 0) {
    //   // Move the arm via variable control
    //   Robot.ArmSubsystem.moveArmBasePercent(OI.getArmBasePercent());
    //   // Update the current angle position
    //   Robot.ArmSubsystem.updateAngle();
    //   // Update the destination angle to where the operator wants it
    //   Robot.ArmSubsystem.updateDAngle();
    //   // Move the arm via PID to the setpoint (either maintain level or continue course if the value was not changed or move to new angle)
    //   Robot.ArmSubsystem.setSetpoint(Robot.ArmSubsystem.angle);
    // }

    Robot.ArmSubsystem.moveArmBasePercent(OI.getArmBasePercent());

    // Move the wrist via variable control
    Robot.ArmSubsystem.moveForeArmPercent(OI.getForeArmPercent());

    if (OI.driveWristUpFixed()) {
      Robot.ArmSubsystem.moveForeArm(true);
    } else if (OI.driveWristDownFixed()) {
      Robot.ArmSubsystem.moveForeArm(false);
    } else {
      Robot.ArmSubsystem.maintainWristLevel();
    }

    // Use intake
    Robot.ArmSubsystem.intake(OI.getIntake(), OI.getOuttake());

    if (OI.groundPickup()) {
      Robot.ArmSubsystem.groundToggle(true);
    } else if (OI.groundEject()) {
      Robot.ArmSubsystem.groundToggle(false);
    } else {
      Robot.ArmSubsystem.stopGroundH();
    }

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
