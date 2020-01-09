/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Add your docs here.
 */
public class WheelSpin extends TimedRobot
 { 
    Talon talon1;
    
    public WheelSpin ()
        { 
            talon1 = new Talon(1);
        }
        public void teleopPeriodic(){
            talon1.set(.5);
        } 
}
