
package frc.robot.commands.positions;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MedGoal extends CommandGroup {
 
  public MedGoal() {
    addSequential(new MoveArm(90));
    addSequential(new MoveWrist(120));
  }
}
