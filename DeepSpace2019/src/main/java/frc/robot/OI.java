
package frc.robot;

import frc.robot.commands.positions.*;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.commands.MoveArm;

/*
 * Operator Interface - Maps controller actions to the actions/capabilities of the robot
 */

public class OI {

    // Driver Controls
    public static final XboxController Driver = new XboxController(0);
    // Operator controls
    public static final XboxController Operator = new XboxController(1);
    
    public static JoystickButton aButton;
    public static JoystickButton bButton;
    public static JoystickButton xButton;
    public static JoystickButton yButton;

    public static JoystickButton lTrigger;
    public static JoystickButton rTrigger;

    public OI () {

        aButton = new JoystickButton(Operator, 1);
        bButton = new JoystickButton(Operator, 2);
        xButton = new JoystickButton(Operator, 3);
        yButton = new JoystickButton(Operator, 4);

        lTrigger = new JoystickButton(Operator, 5);
        rTrigger = new JoystickButton(Operator, 6);

        aButton.whenPressed(new DefaultPosition());
        bButton.whenPressed(new MoveArm(30));
        xButton.whenPressed(new LowGoal());
        yButton.whenPressed(new MedGoal());

        lTrigger.whenPressed(new Intake());
        rTrigger.whenPressed(new Outtake());


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

    public static boolean getIntake () {
        return Operator.getTriggerAxis(Hand.kLeft) > 0.05;
    }

    public static boolean getOuttake () {
        return Operator.getTriggerAxis(Hand.kRight) > 0.05;
    }

    public static boolean activateHatchOp () {
        return Operator.getBumper(Hand.kLeft);
    }

    public static boolean releaseHatchOp () {
        return Operator.getBumper(Hand.kRight);
    }

}
