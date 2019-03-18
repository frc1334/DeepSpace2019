
package frc.robot;

import frc.robot.commands.positions.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/*
 * Operator Interface - Maps controller actions to the actions/capabilities of the robot
 */

public class OI {

    // Driver Controls
    public static final XboxController Driver = new XboxController(0);
    // Operator controls
    public static final XboxController Operator = new XboxController(1);
    
    Button aButton = new JoystickButton(Operator, 0);
    Button bButton = new JoystickButton(Operator, 1);
    Button xButton = new JoystickButton(Operator, 2);
    Button yButton = new JoystickButton(Operator, 3);

    public OI () {
        aButton.whenPressed(new DefaultPosition());
        bButton.whenPressed(new GroundIntake());
        xButton.whenPressed(new LowGoal());
        yButton.whenPressed(new MedGoal());
    }

    // DRIVER

    public static double getDriverSpeed () {
        return Driver.getTriggerAxis(Hand.kRight) - Driver.getTriggerAxis(Hand.kLeft);
    }

    public static double getDriverSteer () {
        return Driver.getRawAxis(0);
    }

    public static boolean unwind () {
        return Driver.getXButton();
    }

    public static boolean climb () {
        return Driver.getBButton();
    }

    public static boolean climbDeploy () {
        return Driver.getAButton();
    }

    // OPERATOR

    public static double getArmBasePercent () {
        return (Operator.getRawAxis(1));
    }

    public static double getForeArmPercent () {
        return (Operator.getRawAxis(5));
    }

    public static boolean positionArm0Deg () {
        return false;
    }

    public static boolean positionArm90Deg () {
        return false;
    }

    public static boolean driveWristUpFixed () {
        return Operator.getAButton();
    }

    public static boolean driveWristDownFixed () {
        return Operator.getBButton(); 
    }  

    public static boolean groundPickup () {
        return Operator.getXButton();
    }

    public static boolean groundEject () {
        return Operator.getYButton();
    }

    public static boolean getIntake () {
        return Operator.getTriggerAxis(Hand.kRight) > 0.05;
    }

    public static boolean getOuttake () {
        // return Operator.getTriggerAxis(Hand.kRight) > 0.05;
        return Operator.getBumper(Hand.kRight);
    }

    public static boolean LowLevel () {
        return Operator.getPOV() == 0;
    }

    public static boolean positionArm135Deg () {
        return Operator.getPOV() == 270;
    }

}
