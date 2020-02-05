/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
/**
 * Add your docs here.
 */
public class Ball {
    public Ball() {
        RobotMap.talonBallShooter.setNeutralMode(NeutralMode.Brake);
    }
    
    public void ballShooter() {
        if (RobotMap.controller.getAButton() && RobotMap.ballButton.get()) {
            intakeBall();

        } else if (RobotMap.controller.getBButton()) {
            shootBall();
            
        }   
        else {
            RobotMap.talonBallShooter.stopMotor();
            RobotMap.talonBallIntake.stopMotor();
        }
      
    }    

    public void intakeBall() {
        RobotMap.talonBallIntake.set(-1);
        RobotMap.talonBallShooter.set(.25);
    }

    public void shootBall() {
        RobotMap.talonBallShooter.set(-1);
        RobotMap.talonBallIntake.set(-1);
        
    }
}
