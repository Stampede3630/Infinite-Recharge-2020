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
        if (RobotMap.CONTROLLER.getPOV() == 0) {
            extend();
        }
        else if (RobotMap.CONTROLLER.getPOV()==180) {
            retract();
        }
        else if (RobotMap.CONTROLLER.getPOV()==270) {
            strafeLeft();
        }
        else if (RobotMap.CONTROLLER.getPOV()==90) {
            strafeRight();
        }
        else {
            RobotMap.ClimbMap.ELEVATOR_SPARK.set(0);
            RobotMap.ClimbMap.TROLLEY_SPARK.set(0);
        }
    }

    public void extend() {
        if (RobotMap.ClimbMap.MAX_LIMIT_SWITCH.get() == false) {
          RobotMap.ClimbMap.ELEVATOR_SPARK.set(elevatorSpeed); 
        }
        else{
            RobotMap.ClimbMap.ELEVATOR_SPARK.set((0));     
           }
    }

    public void retract() {
        if (RobotMap.ClimbMap.MIN_LIMIT_SWITCH.get() == false) {
         RobotMap.ClimbMap.ELEVATOR_SPARK.set((-elevatorSpeed)); 
        }
        else{
         RobotMap.ClimbMap.ELEVATOR_SPARK.set((0));     
        }

    }

    public void strafeLeft() {
        RobotMap.ClimbMap.TROLLEY_SPARK.set(-(strafeSpeed));
    }

    public void strafeRight() {
       RobotMap.ClimbMap.TROLLEY_SPARK.set(strafeSpeed);
    }
}
