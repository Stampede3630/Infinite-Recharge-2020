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
    
    public void ballShooter() {
        if (RobotMap.controller.getAButton()) {
            intakeBall();
        }
        else {
            RobotMap.talonBallIntake.stopMotor();

        }
        if (RobotMap.controller.getBButton()) {
            shootBall();
            
        }
    
        else {
            RobotMap.talonBallShooter.stopMotor();
        }
      
    }    

    public void intakeBall() {
        RobotMap.talonBallIntake.set(-0.5);
        RobotMap.controller.getAButtonReleased();
    }

    public void shootBall() {
        RobotMap.talonBallShooter.set(-0.5);
        RobotMap.controller.getBButton();
    }
}
