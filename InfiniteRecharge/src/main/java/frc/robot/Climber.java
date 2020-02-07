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

    public static final double elevatorSpeed = 0.9;
    public static final double strafeSpeed = 0.5;

    public void climberPeriodic() {
        if (RobotMap.controller.getAButton()) {
            extend();
            System.out.println("A pressed");
        }
        else if (RobotMap.controller.getBButton()) {
            retract();
            System.out.println("B pressed");
        }
        else if (RobotMap.controller.getXButton()) {
            strafeLeft();
            System.out.print("X pressed");
        }
        else if (RobotMap.controller.getYButton()) {
            strafeRight();
            System.out.print("Y pressed");
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
