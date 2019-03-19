
package frc.robot.commands.positions;

import frc.robot.Robot;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DefaultPosition extends CommandGroup {
  
  public DefaultPosition() {
    addSequential(new MoveArm(110));
    addSequential(new MoveWrist(70));
  }
}
