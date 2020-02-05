/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.ILoopable;
import frc.robot.CANLed.*;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  DriveTrain driveTrain;
  Ball ball;
 
  @Override
  public void robotInit() {
  driveTrain = new DriveTrain();
  ball = new Ball();

  for (ILoopable loop : TaskList.FullList) {
    Schedulers.PeriodicTasks.add(loop);
  }

  }

  @Override
  public void robotPeriodic() {
    Schedulers.PeriodicTasks.process(); //for led STRIPS
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    driveTrain.teleOpDrive();
    ball.ballShooter();

  }

  @Override
  public void testPeriodic() {

  }

  
}
