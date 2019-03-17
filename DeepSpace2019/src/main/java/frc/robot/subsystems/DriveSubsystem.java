
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotMap;
import frc.robot.Constants;

/**
 * 
 */

public class DriveSubsystem extends PIDSubsystem {

  // Drivetrain TalonSRX map values (roboRIO port values) - 2 cim
  public TalonSRX Left1 = new TalonSRX(RobotMap.Left1);
  public VictorSPX Left2 = new VictorSPX(RobotMap.Left2);
  public TalonSRX Right1 = new TalonSRX(RobotMap.Right1);
  public VictorSPX Right2 = new VictorSPX(RobotMap.Right2);

  // Compressor
  //public Compressor compressor = new Compressor(RobotMap.Compressor);

  // AHRS - navX mxp - Gyro
  AHRS ahrs;

  // Gyro angle value
  double angle = 0;

  // These values are used to determine Gyro angles
  double post = 0;
  double negt = 0;

  public boolean kSensorPhase = true;
  public boolean isClose = false;
  public boolean still = true;

  public DriveSubsystem () {
    // Calls parent constructor of PIDSubsystem with the parameters: "SubsystemName", kP, kI, kD
    super("DriveSubsystem", Constants.kP, Constants.kI, Constants.kD);
    super.getPIDController().setInputRange(-180.0,  180.0);
    super.getPIDController().setOutputRange(-1.0, 1.0);
    super.getPIDController().setAbsoluteTolerance(Constants.kToleranceDegrees);
    super.getPIDController().setContinuous(true);
  }

  public void initDefaultCommand () {

    // Initialize the Talons so that the Left2 and Right2 Talons will follow the movements of the Left1 and Right 1 Talons
    Left2.set(ControlMode.Follower, RobotMap.Left1);
    Right2.set(ControlMode.Follower, RobotMap.Right1);

    // Configure MagEncoders on the Left1 and Right1 Talons: args (FeedbackDevice, kPIDLoopIdx, TimeOutMS)
    //Left1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, 50);
    //Right1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, 50);

    // Set compressor to a closed loop control
    //compressor.setClosedLoopControl(true);

    // Initialize the AHRS sensor
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      resetGyroAngle();
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instancing navX MXP: " + ex.getMessage(), true);
    }
  
  }

  public void initTalonFollower () {
    Left2.set(ControlMode.Follower, RobotMap.Left1);
    Right2.set(ControlMode.Follower, RobotMap.Right1);
  }

  public void CompressorControl(){
		//compressor.setClosedLoopControl(true);
	}

  public void initAutoCommand () {

    // Initialize the Talons so that the Left2 and Right2 Talons will follow the movements of the Left1 and Right 1 Talons
    Left2.set(ControlMode.Follower, RobotMap.Left1);
    Right2.set(ControlMode.Follower, RobotMap.Right1);

    // Configure MagEncoders on the Left1 and Right1 Talons: args (FeedbackDevice, kPIDLoopIdx, TimeOutMS)
    //Left1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, 50);
    //Right1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, 50);

    // Make sure encoder output is positive
    Left1.setSensorPhase(false);
    Right1.setSensorPhase(false);
    
    // Initialize the AHRS sensor
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      resetGyroAngle();
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instancing navX MXP: " + ex.getMessage(), true);
    }

    Left1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,10, Constants.kTimeoutMs);
		Left1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		Right1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,10,Constants.kTimeoutMs);
		Right1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		// set the peak and nominal outputs, 12V means full
		Left1.configNominalOutputForward(0, Constants.kTimeoutMs);
		Left1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		Left1.configPeakOutputForward(1, Constants.kTimeoutMs);
		Left1.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		Right1.configNominalOutputForward(0, Constants.kTimeoutMs);
		Right1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		Right1.configPeakOutputForward(1, Constants.kTimeoutMs);
		Right1.configPeakOutputReverse(-1, Constants.kTimeoutMs);
  
  }

  // Sets the quadrature (relative) sensor to match absolute
  public void setQuadAbsolute (int lAbsolutePos, int rAbsolutePos) {
    Left1.setSelectedSensorPosition(lAbsolutePos, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    Right1.setSelectedSensorPosition(rAbsolutePos, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  public void invertLeftTalons () {
    // Invert the Left Talons
    Left1.setInverted(true);
  }

  // Method to set the Drive Train to drive with PID
  public void setPIDDrive () {
    Left1.set(ControlMode.Position, 0);
    Right1.set(ControlMode.Position, 0);
  }

  public double GyroDrive (double turn) {
    angle += turn;
    int b = (int) angle/180;
    angle = (float) (angle * Math.pow(-1, b));
    return angle;
  }
  
  // Method to change gyro angle to 0 degrees
  public void resetGyroAngle () {
	  ahrs.reset();
	  angle = 0;
	}

  // Method to set the Drive Train to drive normally (without PID)
  public void setRegularDrive () {
    Left1.set(ControlMode.Velocity, 0);
    Right1.set(ControlMode.Velocity, 0);
  }

  // Basic Tank Drive Method
  public void TankDrive(double left, double right){
    if (Constants.kCurrentLimited) {
      // Limit speed if current limiting is true
      left *= 0.6;
      right *= 0.6;
    }
    Left1.set(ControlMode.PercentOutput, left * 0.8);
    Left2.set(ControlMode.PercentOutput, -left * 0.8);
    Right1.set(ControlMode.PercentOutput, -right * 0.8);
    Right2.set(ControlMode.PercentOutput, right * 0.8);
  }
  
  // Basic Arcade Drive Method
  public void ArcadeDrive (double speed, double turn) {
		TankDrive(speed - turn, - speed - turn);
  }

  // Method that "Soft-shifts" the Talons
  public void changeCurrentLimit () {
    Constants.kCurrentLimited = !Constants.kCurrentLimited;
  }

  // Method that returns the PID value of the gyro sensor
  protected double returnPIDInput () {
    return ahrs.pidGet();
  }

  // Method that adjusts the Gyro according to the PID outputs
  public void usePIDOutput (double output) {

    if (output > Constants.kMaxGyro) {
      Constants.kGyroRotationRate = Constants.kMaxGyro;
    } else if (output <= -Constants.kMaxGyro) {
      Constants.kGyroRotationRate = -Constants.kMaxGyro;
    } else {
      // In between maximum and minimum
      Constants.kGyroRotationRate = output;
    }

    if (still) {
      if (this.getPIDController().getError()>=2 || this.getPIDController().getError()<=-2) {
				if (Constants.kGyroRotationRate<= Constants.kMinimalVoltage && Constants.kGyroRotationRate > 0) {
					post += 1;
					Constants.kGyroRotationRate = Constants.kMinimalVoltage - ((1-this.getPIDController().getError())/65)+post/100;
				} else if( Constants.kGyroRotationRate>= -Constants.kMinimalVoltage && Constants.kGyroRotationRate <0) {
					negt+= 1;
					Constants.kGyroRotationRate = -Constants.kMinimalVoltage + ((1- this.getPIDController().getError())/65)-negt/100;
        }
			} else {
				post = 0;
				negt = 0;
			}
    }

  }

}
