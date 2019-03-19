
package frc.robot.commands.positions;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LowGoal extends CommandGroup {
  
  public LowGoal() {
    addSequential(new MoveArm(22));
    addSequential(new MoveWrist(158));
  }

}
