
package frc.robot.commands.positions;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DefaultPosition extends CommandGroup {
  
  public DefaultPosition() {
    System.out.println("Default Position");
    addSequential(new MoveArm(40));
  }
}
