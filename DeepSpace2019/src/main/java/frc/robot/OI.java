
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/*
 * Operator Interface - Maps controller actions to the actions/capabilities of the robot
 */

public class OI {

    public static final XboxController Driver = new XboxController(0);
    public static final XboxController Operator = new XboxController(1);

    public static double getDriverSpeed () {
        return Driver.getTriggerAxis(Hand.kRight) - Driver.getTriggerAxis(Hand.kLeft);
    }

    public static double getDriverSteer () {
        return Driver.getRawAxis(0);
    }

    public static boolean changeDriverCurrentLimit () {
        return Driver.getAButton();
    }

}
