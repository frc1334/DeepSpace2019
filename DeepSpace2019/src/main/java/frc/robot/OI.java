
package frc.robot;

import frc.robot.utils.Xbox360Controller;

/**
 * Operator Interface - Maps the controllers to the actions/capabilities of the robot
 */

public class OI {

    public static final Xbox360Controller Driver = new Xbox360Controller(0, 0.15);
    public static final Xbox360Controller Operator = new Xbox360Controller(1, 0.15);

}
