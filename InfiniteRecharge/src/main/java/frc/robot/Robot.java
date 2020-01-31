package frc.robot;

import com.ctre.phoenix.ILoopable;

import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.CANLed.*;

public class Robot extends TimedRobot 
{
 
  public static ShuffleboardTab tab;
  public static DriveTrain driveTrain;
  public static Manipulator manipulator;
  public static Diagnostics diagnostics;
  public static Choosers choosers;
  public static PathChooser pathChooser = new PathChooser();
  public static NetworkTableEntry pathSelected, hatchBallSelected, hatchLeft, hatchRight;
  public static boolean isDeleted = false;

  @Override
  public void robotInit() 
  {
    driveTrain = new DriveTrain();
    manipulator = new Manipulator();
    diagnostics = new Diagnostics();
    choosers = new Choosers(driveTrain, manipulator);

    tab = Shuffleboard.getTab("driverTab");
    pathSelected = tab.add("PathSelected", "").withWidget("PathSelector").withSize(7,6).withPosition(3, 0).getEntry();
    hatchBallSelected = tab.add("hatchBallSelected","").withWidget("BigButtonsWidget").withSize(3,2).withPosition(0,0).getEntry();

    //hatchInBoolean = tab.add("Hatch In",false).withSize(3,2).withPosition(0,3).getEntry();
    hatchLeft = tab.add("Hatch in L", choosers.robotMap.hatchButton.get()).withPosition(11,0).withSize(1,1).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    hatchRight = tab.add("Hatch in R", choosers.robotMap.dumbHatchButton.get()).withPosition(11,1).withSize(1,1).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    for (ILoopable loop : TaskList.FullList) {
			Schedulers.PeriodicTasks.add(loop);
    }
    

  }
  
  @Override
  public void robotPeriodic() 
  {
    
    choosers.letterButtons();

    diagnostics.toSmartDashboard();

    pathChooser.stringToPath(pathSelected.getString(""));

    diagnostics.getForwardMode();
    
    diagnostics.ultrasonicSensorReading(); 

    Schedulers.PeriodicTasks.process();
    
    diagnostics.limelightValues();

    choosers.updatePath();
    
    choosers.setManipulatorMode();

    diagnostics.resetGyro();

    hatchRight.setBoolean(choosers.robotMap.hatchButton.get());
    hatchLeft.setBoolean(choosers.robotMap.dumbHatchButton.get());

  }

  
  @Override
  public void autonomousInit() 
  {

    manipulator.robotMap.ahrs.reset();
  
  }

  
  @Override
  public void autonomousPeriodic() 
  {

    choosers.setDriveMode();

    driveTrain.drive();

    manipulator.manipulatorPeriodic();

  }

  @Override
  public void teleopInit() 
  {
  }

  @Override
  public void teleopPeriodic() 
  {

    choosers.setDriveMode();

    driveTrain.drive();

    manipulator.manipulatorPeriodic();

  }

 
  @Override
  public void testPeriodic() 
  {



  }

  @Override
  public void disabledInit() 
  {
    
  }


  //Untested
  @Override
  public void disabledPeriodic()
  {

    SmartDashboard.putBoolean("Big Red Button", isDeleted);
    if (isDeleted) 
    {
      pathSelected.delete();
      hatchBallSelected.delete();

      pathSelected = tab.add("PathSelected", "").withWidget("PathSelector").withSize(7,6).withPosition(3, 0).getEntry();
      hatchBallSelected = tab.add("hatchBallSelected","").withWidget("BigButtonsWidget").withSize(3,2).withPosition(0,0).getEntry();

      isDeleted = false;
    }

  }
}