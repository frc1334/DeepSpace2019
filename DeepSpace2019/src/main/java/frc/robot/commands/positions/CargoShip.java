
package frc.robot.commands.positions;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoShip extends CommandGroup {

  public CargoShip() {
    addSequential(new MoveArm(90));
    addSequential(new MoveWrist(120));
  }

}
