/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Ball {
    RobotMap robotMap;
    
    public void ballShooter() {
        if (robotMap.controller.getAButtonPressed()) {
            intakeBall();
        }
        if (robotMap.controller.getBButtonPressed()) {
            shootBall();
        }
    }    

    public void intakeBall() {
        robotMap.talonBallIntake.set(0.5);
       
    }

    public void shootBall() {
        robotMap.talonBallShooter.set(0.5);
        robotMap.controller.getBButton();
    }
}
