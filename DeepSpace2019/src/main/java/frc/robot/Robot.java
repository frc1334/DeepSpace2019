
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.OI;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * Ben told me to remember this number for some reason: 32677
 */

public class Robot extends TimedRobot {
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static DriveSubsystem DriveSubsystem = new DriveSubsystem();
  public static ArmSubsystem ArmSubsystem = new ArmSubsystem();
  public static WristSubsystem WristSubsystem = new WristSubsystem();
  public static IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  public static ClimberSubsystem ClimberSubsystem = new ClimberSubsystem();

  public static OI OI = new OI();

  public static DriveCommand DriveCommand;

  public static NetworkTable Pitable;

  public static NetworkTableEntry TapeYaw;
  public static NetworkTableEntry TapeDetected;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  public void robotInit() {

    DriveCommand = new DriveCommand();

    // Vision

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    Pitable = inst.getTable("networkTable");

    TapeDetected = Pitable.getEntry("Tape");

    TapeDetected.setBoolean(true);

    // TapeYaw = Pitable.getEntry("tapeYaw");
    // CargoYaw = Pitable.getEntry("cargoYaw");

    // // Camera stream source: http://roborio-TEAM-frc.local:1181/?action=stream

    // UsbCamera Camera = CameraServer.getInstance().startAutomaticCapture(0);
    // Camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 640, 360, 30);
    // Camera.setResolution(640, 480);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
 
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   *  <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */

  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is call
   * ed periodically during autonomous.
   */

  public void autonomousPeriodic() {

    Scheduler.getInstance().run();

    DriveCommand.start();
    
  }

  /**
   * This function is called periodically during operator control.
   */

  public void teleopPeriodic() {

    System.out.println(ArmSubsystem.getCurrentAngle());
    System.out.println(WristSubsystem.getCurrentAngle());

    SmartDashboard.putNumber("Arm Angle", ArmSubsystem.getCurrentAngle());
    SmartDashboard.putNumber("Wrist Angle", WristSubsystem.getCurrentAngle());

    Scheduler.getInstance().run();

    DriveCommand.start();
    
  }

  /**
   * This function is called periodically during test mode.
   */

  public void testPeriodic() {
  }
}
