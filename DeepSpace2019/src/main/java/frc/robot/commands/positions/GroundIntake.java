
package frc.robot.commands.positions;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GroundIntake extends CommandGroup {
 
  public GroundIntake() {
    addSequential(new MoveArm(30));
    addSequential(new MoveWrist(135));
  }

}
