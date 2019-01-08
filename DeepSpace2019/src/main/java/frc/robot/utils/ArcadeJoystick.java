
package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */

public class ArcadeJoystick extends Joystick {

    // Default deadzone (0.15)
    private double deadzone = 0.15;
    
    public ArcadeJoystick (int port, double deadzone) {
        super(port);
        this.deadzone = deadzone;
    }

    public double getXAxis () {
        return deadzone(this.getRawAxis(0)); 
    }

	public double getYAxis () {
        return deadzone(this.getRawAxis(1)); 
    }

    public double deadzone (double in, double deadzone) {
        return ((Math.abs(in) <= deadzone) ? 0 : in); 
    }

	public double deadzone (double in) {
        return deadzone(in, this.deadzone); 
    }

}
