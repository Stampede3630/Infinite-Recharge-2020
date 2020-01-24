/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.revrobotics.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  WheelSpin spinThing;
  SpeedControllerGroup rightyRobo;
  SpeedControllerGroup leftyRobo;
  Talon talon0;
  Talon talon1; 
  Talon talon2;
  Talon talon3;
  DifferentialDrive roboGo;
  XboxController remoty;
  PIDController piddie;
  AHRS veel;
  AVA enCod;
  ColorSensorV3 colo; 
  Color noColor; 
  Ultrasonic BIGsonny;
  Ultrasonic BIGsonny1;
  Balldigestion testing; 
  



  

  public Robot ()
  {
    //spinThing = new WheelSpin();
    /*
    enCod = new AVA();
    talon0 = new Talon(0);
    talon1 = new Talon(1);
    talon3 = new Talon(3);
    talon2 = new Talon(2);
    rightyRobo = new SpeedControllerGroup(talon0, talon1); 
    leftyRobo = new SpeedControllerGroup (talon3, talon2);
    roboGo = new DifferentialDrive(leftyRobo,rightyRobo);
    remoty = new XboxController(0); 
    piddie = new PIDController (0.01,0,0);
    veel = new AHRS(SPI.Port.kMXP); 
    */
    //colo = new ColorSensorV3(I2C.Port.kOnboard );
    //BIGsonny = new Ultrasonic(8,9);
    //BIGsonny1 = new Ultrasonic(6, 7);
    
  
   // BIGsonny.setAutomaticMode(true);
    //BIGsonny.setDistanceUnits(Ultrasonic.Unit.kInches);
    testing = new Balldigestion();
    
  } 
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
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
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //spinThing.teleopPeriodic();
    //roboGo.arcadeDrive(xSpeed, zRotation);
  //roboGo.arcadeDrive(remoty.getY(Hand.kLeft),remoty.getX(Hand.kRight));
  //piddie.calculate(veel.getAngle(),45);
//roboGo.arcadeDrive(0,piddie.calculate(veel.getAngle(),45));
//noColor = colo.getColor();

//roboGo.arcadeDrive(piddie.calculate(enCod.piggie(),48), 0); //piddie.calculate(veel.getAngle(),0));
//SmartDashboard.putNumber("pig",piddie.calculate(enCod.piggie(),48 ));
//SmartDashboard.putNumber("string", colo.getIR());
//SmartDashboard.putNumber("hey", noColor.green); doesnt work as well as stringy
//SmartDashboard.putNumber("stringy", colo.getRawColor().green);
//SmartDashboard.putNumber("proximity", colo.getGreen());
//SmartDashboard.putNumber("ultrasonic", BIGsonny.getRangeInches());
//SmartDashboard.putNumber("ultrasonic", BIGsonny1.getRangeInches());
testing.greggory(); 
  }

    
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
public void Portie() 
{
//colo.getColor();
//colo.getRawColor();
//colo.getIR();
}

}

