
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/*
 * Operator Interface - Maps controller actions to the actions/capabilities of the robot
 */

public class OI {

    public static final XboxController Driver = new XboxController(0);
    public static final XboxController Operator = new XboxController(1);

    // DRIVER

    public static double getDriverSpeed () {
        return Driver.getTriggerAxis(Hand.kRight) - Driver.getTriggerAxis(Hand.kLeft);
    }

    public static double getDriverSteer () {
        return Driver.getRawAxis(0);
    }

    public static boolean changeDriverCurrentLimit () {
        return Driver.getXButton();
    }

    public static boolean deployClimber () {
        return Driver.getAButton();
    }

    public static boolean climberClamp () {
        return Driver.getBButton();
    }

    public static boolean climberBackFlip () {
        return Driver.getYButton();
    }

    // OPERATOR

    public static double getArmBasePercent () {
        return Operator.getRawAxis(1);
    }

    public static double getForeArmPercent () {
        return Operator.getRawAxis(5);
    }

    public static boolean groundPickUp () {
        return Operator.getAButton();
    }

    public static boolean LowLevel () {
        return Operator.getBButton();
    }

    public static boolean getIntake () {
        return Operator.getTriggerAxis(Hand.kLeft) > 0.15;
    }

    public static boolean getOuttake () {
        return Operator.getTriggerAxis(Hand.kRight) > 0.15;
    }

    public static boolean positionArm0Deg () {
        return Operator.getPOV() == 90;
    }

    public static boolean positionArm90Deg () {
        return Operator.getPOV() == 0;
    }

    public static boolean positionArm180Deg () {
        return Operator.getPOV() == 270;
    }

}
