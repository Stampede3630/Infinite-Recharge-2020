/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Climber {

    public static final double elevatorSpeed = 0.9;
    public static final double strafeSpeed = 0.5;

    public void climberPeriodic() {
        if (RobotMap.controller.getPOV() == 0) {
            extend();
        }
        else if (RobotMap.controller.getPOV()==180) {
            retract();
        }
        else if (RobotMap.controller.getPOV()==270) {
            strafeLeft();
        }
        else if (RobotMap.controller.getPOV()==90) {
            strafeRight();
        }
        else {
            RobotMap.elevatorSpark.set(0);
            RobotMap.trolleySpark.set(0);
        }
    }

    public void extend() {
        if (RobotMap.maxLimitSwitch.get() == false) {
          RobotMap.elevatorSpark.set(elevatorSpeed); 
        }
        else{
            RobotMap.elevatorSpark.set((0));     
           }
    }

    public void retract() {
        if (RobotMap.minLimitSwitch.get() == false) {
         RobotMap.elevatorSpark.set((-elevatorSpeed)); 
        }
        else{
         RobotMap.elevatorSpark.set((0));     
        }

    }

    public void strafeLeft() {
        RobotMap.trolleySpark.set(-(strafeSpeed));
    }

    public void strafeRight() {
       RobotMap.trolleySpark.set(strafeSpeed);
    }
}