/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.DriveMap;
import frc.robot.RobotMap.DrivetrainMap;

/**
 * Add your docs here.
 */
public class RightTrench {
    private Timer autoTime = new Timer();
    private double timeThreshold = 8;
    private double shooterWait = 0;
    private PIDController autoDistancePID = new PIDController(1, 0, 0);
    private double meterSetpoint = 1;
    private int step = 0;
    
  
    public void resetAutoTime()
    {
        autoTime.reset();
        autoTime.start();
    }
    public void periodicOld()
    {
        if(RobotMap.AutoBooleans.SHOOT_NOW && (autoTime.get() > timeThreshold))
        {
            RobotMap.AutoBooleans.SHOOT_NOW = false;
        }
        else if(RobotMap.AutoBooleans.SHOOT_NOW)
        {
            Drivetrain.getInstance().drive(0, 0, TargetAlignDrive.getInstance().align()*RobotMap.DriveMap.MAX_SPEED, true);
        } 
        else
        {
            //System.out.println("Trying to move");
            double xSpeed = autoDistancePID.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX(), meterSetpoint);
            Drivetrain.getInstance().drive(xSpeed * RobotMap.DriveMap.MAX_SPEED, 0, 0, true);
        }
    }

    public void threeBallPeriodic()
    {
        switch(step)
        {
            case 0:
            autoTime.reset();
            autoTime.start();
            RobotMap.DrivetrainMap.ODOMETRY.resetPosition(new Pose2d(-3.05,-2.4, new Rotation2d(Drivetrain.getInstance().getAngle().getRadians())), new Rotation2d(Drivetrain.getInstance().getAngle().getRadians()));
            RobotMap.AutoBooleans.SHOOT_NOW = true;
            step++;
            break;
            
            case 1:
            Drivetrain.getInstance().drive(0, 0, TargetAlignDrive.getInstance().align()*RobotMap.DriveMap.MAX_SPEED, true);
            
            if(RobotMap.AutoBooleans.SHOOT_NOW && (autoTime.get() > timeThreshold))
            {
                RobotMap.AutoBooleans.INTAKE_NOW = true;
                RobotMap.AutoBooleans.SHOOT_NOW = false;
                step++;
        
            }
            break;

            case 2:
            Drivetrain.getInstance().spoteGo(-6.166, -.704, 0);

            if(Drivetrain.getInstance().canMoveOn())
            {
                step++;
            }

            //double xSpeed = autoDistancePID.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX(), meterSetpoint);
            //Drivetrain.getInstance().drive(xSpeed * RobotMap.DriveMap.MAX_SPEED, 0, 0, true);
            //if(Math.abs(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX() -  meterSetpoint) < 0.05)
            break;

            case 3: 
            //double rotationSpeed = autoDistancePID.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getRotation().getDegrees(), 0);
            Drivetrain.getInstance().spoteGo(-7.99, -.704, 0);
            if(Drivetrain.getInstance().canMoveOn())
            {
                RobotMap.AutoBooleans.INTAKE_NOW = false;
                RobotMap.AutoBooleans.SHOOT_NOW = true;
                autoTime.reset();
                autoTime.start();
                step++;
            }
            break;

            case 4:
            Drivetrain.getInstance().drive(0, 0, TargetAlignDrive.getInstance().align()*RobotMap.DriveMap.MAX_SPEED, true);
            
            if(RobotMap.AutoBooleans.SHOOT_NOW && (autoTime.get() > timeThreshold))
            {
                RobotMap.AutoBooleans.INTAKE_NOW = true;
                RobotMap.AutoBooleans.SHOOT_NOW = false;
                step++;
            }
            break;

            default:
            Drivetrain.getInstance().drive(0,0,0,RobotMap.StateChooser.FIELD_RELATIVE);
            break;
    }
    
   



    
}
public void twoBallPeriodic()
{
    switch(step)
    {
        case 0:
        autoTime.reset();
        autoTime.start();
        RobotMap.DrivetrainMap.ODOMETRY.resetPosition(new Pose2d(-3.05,-8.21, new Rotation2d(Drivetrain.getInstance().getAngle().getRadians())), new Rotation2d(Drivetrain.getInstance().getAngle().getRadians()));
        RobotMap.AutoBooleans.SHOOT_NOW = false;
        RobotMap.AutoBooleans.INTAKE_NOW = true;
        step++;
        break;

        case 1:
        Drivetrain.getInstance().spoteGo(-6.36, -7.3, 0);

        if(Drivetrain.getInstance().canMoveOn())
        {
            step++;
        }
        break;

        case 2:
        Drivetrain.getInstance().spoteGo(-6.36, -7.74, 0);

        if(Drivetrain.getInstance().canMoveOn())
        {
            RobotMap.AutoBooleans.INTAKE_NOW = false;
            step++;
        }
        break;

        case 3:
        Drivetrain.getInstance().spoteGo(-3.05,-4.11, -.65);

        if(Drivetrain.getInstance().canMoveOn())
        {
            autoTime.reset();
            autoTime.start();
            RobotMap.AutoBooleans.INTAKE_NOW = false;
            RobotMap.AutoBooleans.SHOOT_NOW = true;
            step++;
        }
        break;

        case 4:
        Drivetrain.getInstance().drive(0, 0, TargetAlignDrive.getInstance().align()*RobotMap.DriveMap.MAX_SPEED, true);
        
        if(RobotMap.AutoBooleans.SHOOT_NOW && (autoTime.get() > timeThreshold))
        {
            RobotMap.AutoBooleans.INTAKE_NOW = false;
            RobotMap.AutoBooleans.SHOOT_NOW = false;
            step++;
        }
        break;

        default:
        Drivetrain.getInstance().drive(0,0,0,RobotMap.StateChooser.FIELD_RELATIVE);
        break;
} 
}



}

