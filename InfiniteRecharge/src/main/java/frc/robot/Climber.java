/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Climber {

    public static final double elevatorSpeed = 0.5;
    public static final double strafeSpeed = 0.5;

    public void climberPeriodic() {
        if (RobotMap.controller.getAButtonPressed()) {
            extend();
        }
        if (RobotMap.controller.getBButtonPressed()) {
            retract();
        }
        if (RobotMap.controller.getXButton()) {
            strafeLeft();
        }
        if (RobotMap.controller.getYButton()) {
            strafeRight();
        }
    }

    public void extend() {
        if (RobotMap.elevatorMaxExtension.get() == false) {
            RobotMap.elevatorSpark.set(elevatorSpeed); 
        }
       
    }

    public void retract() {
        if (RobotMap.elevatorMinExtension.get() == false) {
            RobotMap.elevatorSpark.set(-(elevatorSpeed)); 
        }
    }

    public void strafeLeft() {
        RobotMap.trolleySpark.set(-(strafeSpeed));
    }

    public void strafeRight() {
        RobotMap.trolleySpark.set(strafeSpeed);
    }
}
