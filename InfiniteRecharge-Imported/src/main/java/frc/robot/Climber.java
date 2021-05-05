/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Climber {

    private static Climber instance;
    public boolean shuffleboardClimberOn = false;

    static {
        instance = new Climber();
    }

    public static Climber getInstance() {
        return instance;
    }

    public static enum sClimberState {
        STOPPED, LEFT, RIGHT, UP, DOWN
    }

    private SendableChooser<sClimberState> sClimber;

    private Climber() // private to make sure it isn't instantiated elsewhere
    {
        sClimber = new SendableChooser<sClimberState>();
        SmartDashboard.putData(sClimber);

        sClimber.setDefaultOption("Stopped", sClimberState.STOPPED);
        sClimber.addOption("Left", sClimberState.LEFT);
        sClimber.addOption("Right", sClimberState.RIGHT);
        sClimber.addOption("Up", sClimberState.UP);
        sClimber.addOption("Down", sClimberState.DOWN);

        sClimberState sClimbState;
    }

    public void climberPeriodic() {
        if (RobotMap.CONTROLLER.getPOV() == 0) {
            extend();
            SmartDashboard.putBoolean("Climber up", true);
        } else if (RobotMap.CONTROLLER.getPOV() == 180) {
            retract();
            SmartDashboard.putBoolean("Climber down", true);
        } else if (RobotMap.CONTROLLER.getPOV() == 270) {
            strafeLeft();
            SmartDashboard.putBoolean("Climber left", true);
        } else if (RobotMap.CONTROLLER.getPOV() == 90) {
            strafeRight();
            SmartDashboard.putBoolean("Climber right", true);
        } else {
            RobotMap.ClimberMap.ELEVATOR_SPARK.set(0);
            RobotMap.ClimberMap.TROLLEY_SPARK.set(0);
            SmartDashboard.putBoolean("Climber up", false);
            SmartDashboard.putBoolean("Climber down", false);
            SmartDashboard.putBoolean("Climber left", false);
            SmartDashboard.putBoolean("Climber right", false);
        }
    }

    public void climberDebug() { // 1/28/2021, delete later
        SmartDashboard.putNumber("Climber POV", RobotMap.CONTROLLER.getPOV());
        if (RobotMap.CONTROLLER.getPOV() == 0) {
            SmartDashboard.putBoolean("Climber up", true);
        } else if (RobotMap.CONTROLLER.getPOV() == 180) {
            SmartDashboard.putBoolean("Climber down", true);
        } else if (RobotMap.CONTROLLER.getPOV() == 270) {
            SmartDashboard.putBoolean("Climber left", true);
        } else if (RobotMap.CONTROLLER.getPOV() == 90) {
            SmartDashboard.putBoolean("Climber right", true);
        } else {
            SmartDashboard.putBoolean("Climber up", false);
            SmartDashboard.putBoolean("Climber down", false);
            SmartDashboard.putBoolean("Climber left", false);
            SmartDashboard.putBoolean("Climber right", false);
        }
    }

    public void extend() {
        if (RobotMap.ClimberMap.MAX_LIMIT_SWITCH.get() == false) {
            RobotMap.ClimberMap.ELEVATOR_SPARK.set(RobotMap.ClimberMap.ELEVATOR_SPEED);
        } else {
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

    public void setClimberOn() {
        if (SmartDashboard.getBoolean("Shuffleboard Climber Control", true)) {
            shuffleboardClimberOn = true;
        } else {
            shuffleboardClimberOn = false;
        }
    }

    public void putClimberOn() {
        SmartDashboard.putBoolean("Shuffleboard Climber Control", false);

    }

    public void shuffleboardClimber() {
        setClimberOn();

        if (shuffleboardClimberOn == true) {
            if (sClimber.getSelected() == sClimberState.LEFT) {
                strafeLeft();
            } else if (sClimber.getSelected() == sClimberState.RIGHT) {
                strafeRight();
            } else if (sClimber.getSelected() == sClimberState.UP) {
                extend();
            } else if (sClimber.getSelected() == sClimberState.DOWN) {
                retract();
            } else if (sClimber.getSelected() == sClimberState.STOPPED) {

            }
        } else {
        }
    }

}
