/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Climber {

    private static Climber instance;

    static {
      instance = new Climber();
    }
  
    public static Climber getInstance() {
      return instance;
    }

    private Climber() // private to make sure it isn't instantiated elsewhere
    {

    }


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
            RobotMap.ClimberMap.ELEVATOR_SPARK.set(0);
            RobotMap.ClimberMap.TROLLEY_SPARK.set(0);
        }
    }

    public void extend() {
        if (RobotMap.ClimberMap.MAX_LIMIT_SWITCH.get() == false) {
          RobotMap.ClimberMap.ELEVATOR_SPARK.set(RobotMap.ClimberMap.ELEVATOR_SPEED); 
        }
        else{
            RobotMap.ClimberMap.ELEVATOR_SPARK.set((0));     
           }
    }

    public void retract() {
         RobotMap.ClimberMap.ELEVATOR_SPARK.set(-RobotMap.ClimberMap.ELEVATOR_SPEED); 

    }

    public void strafeLeft() {
        RobotMap.ClimberMap.TROLLEY_SPARK.set(-RobotMap.ClimberMap.STRAFE_SPEED);
    }

    public void strafeRight() {
       RobotMap.ClimberMap.TROLLEY_SPARK.set(RobotMap.ClimberMap.STRAFE_SPEED);
    }
}
