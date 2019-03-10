
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/*
 * Operator Interface - Maps controller actions to the actions/capabilities of the robot
 */

public class OI {

    public static final XboxController Driver = new XboxController(0);
    
    public static final XboxController Operator = new XboxController(1);

    // DRIVER

    public static double getDriverSpeed () {
        return Driver.getTriggerAxis(Hand.kLeft) - Driver.getTriggerAxis(Hand.kRight);
    }

    public static double getDriverSteer () {
        return Driver.getRawAxis(0);
    }

    public static boolean changeDriverCurrentLimit () {
        return Driver.getXButton();
    }

    public static boolean climberDeploy () {
        return Driver.getBButton();
    }

    public static boolean climb () {
        return Driver.getYButton();
    }

    // OPERATOR

    public static double getArmBasePercent () {
        return Operator.getRawAxis(1) * -0.5;
    }

    public static double getForeArmPercent () {
        return Operator.getRawAxis(5) * -0.5;
    }

    public static boolean positionArm0Deg () {
        return Operator.getAButton();
    }

    public static boolean positionArm90Deg () {
        return Operator.getBButton();
    }

    public static boolean getIntake () {
        return Operator.getTriggerAxis(Hand.kLeft) > 0.15;
    }

    public static boolean getOuttake () {
        return Operator.getTriggerAxis(Hand.kRight) > 0.15;
    }

    public static boolean groundPickup () {
        return Operator.getPOV() == 90;
    }

    public static boolean LowLevel () {
        return Operator.getPOV() == 0;
    }

    public static boolean positionArm135Deg () {
        return Operator.getPOV() == 270;
    }

}
