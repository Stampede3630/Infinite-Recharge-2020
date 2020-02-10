/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser<V>;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */

public class IntakeIndex {
    SendableChooser<Integer> intakeChooser;

    XboxController controller;
    WPI_TalonSRX intakeWheels;// first spin wheel
    WPI_TalonSRX pinwheel; // from box to belt
    WPI_TalonSRX belt;
    DoubleSolenoid armsSolenoid; // lowers the arms
    Timer timer;
    Ultrasonic ultrasonic; // on the ground of the belt box
    ColorSensorV3 colorSensorMid;
    ColorSensorV3 colorSensorHigh;
    Shooter shoot;
    double threshold;
    double thresholdWeak;
    boolean bottom, middle, top, none, middleWeak;
    private boolean newMattyState = false;

    private double beltForwardOne = -.32;
    private double beltBackwardsOne = .32;
    private double beltForwardTwo = -.5;
    private double beltBackwardsTwo = .5;

    public IntakeIndex() {
        controller = new XboxController(0);
        intakeWheels = new WPI_TalonSRX(9);
        armsSolenoid = new DoubleSolenoid(2, 3); // 2 solenoid on r
        belt = new WPI_TalonSRX(11);
        timer = new Timer();
        colorSensorHigh = new ColorSensorV3(I2C.Port.kMXP);
        pinwheel = new WPI_TalonSRX(10);
        ultrasonic = new Ultrasonic(5, 3);
        colorSensorMid = new ColorSensorV3(I2C.Port.kOnboard);
        ultrasonic.setAutomaticMode(true);
        ultrasonic.setDistanceUnits(Ultrasonic.Unit.kInches);
        // colorSensorMid.setAutomaticMode(true);
        // colorSensorMid.setDistanceUnits(Ultrasonic.Unit.kInches);

        intakeChooser = new SendableChooser<Integer>();
        intakeChooser.addDefault("Automatic", 1);
        intakeChooser.addObject("Manual", 0);

        belt.setNeutralMode(NeutralMode.Brake);
    }

    public void updateBooleans() {
        threshold = 40000;
        thresholdWeak = 15000;

        bottom = false;
        middle = false;
        top = false;
        none = false;
        middleWeak = false;

        if (ultrasonic.getRangeInches() > 200 || ultrasonic.getRangeInches() < 6) {
            bottom = true;

        }
        if (colorSensorHigh.getGreen() > threshold) {
            top = true;

        }
        if (colorSensorMid.getGreen() > threshold) {
            middle = true;
        }
        if (ultrasonic.getRangeInches() < 200 && ultrasonic.getRangeInches() > 20) {
            none = true;
        }
        if (colorSensorMid.getGreen() > thresholdWeak) {
            middleWeak = true;
        }
    }

    public void intakeChooser(boolean choice) {
        if (choice) {
            index();
        } else {
            manualControl();
        }
    }

    public void manualControl() {
        if (RobotMap.controller.getAButton()) {
            intakeWheels.set(.75);
            pinwheel.set(-.4);
        } else {
            intakeWheels.set(0);
            pinwheel.set(0);
        }

        if (RobotMap.controller.getTriggerAxis(Hand.kRight) > 0.5) {
            belt.set(-.7);
        } else if (RobotMap.controller.getBumper(Hand.kRight)) {
            belt.set(0.7);
        } else {
            belt.set(0);
        }

    }

    public void index() { // most recent intake machine
        // System.out.println(pinwheel.get());
        // updateBooleans();

        if (controller.getAButton()) {
            System.out.println("tester");
            timer.reset();
            timer.start();
            intakeWheels.set(.5);
        }

        else {
            intakeWheels.set(0);
            // System.out.print(timer.get());
        }

        if (timer.get() > 1.5 || bottom || timer.get() == 0) { // if its been 1.5 sec or there's something in the bottom
            pinwheel.set(0);
        } else {
            pinwheel.set(-.8);
        }

        // BELT STUFF!!!!!!!!!!!!!!!!!
        if (controller.getBButton()) {
            armsSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        if (RobotMap.controller.getTriggerAxis(Hand.kRight) > .6 // if shooter up to speed
                && RobotMap.leftShooterFalcon.getSelectedSensorVelocity() >= Shooter.rpmToRotatPer100Mili(Shooter.rotpm)
                        * Shooter.kEncoderUnitsPerRev) {
            belt.set(beltForwardTwo);
            System.out.println("shooter up to speed");

        } else if (top && middle) {
            belt.set(0);
            System.out.println("yes top, yes middle");
        } else if (top && middleWeak) {
            belt.set(0);
            System.out.println("top, middle weak");
        } else if (top && !middle) {
            belt.set(beltBackwardsOne);
            System.out.println("yes top, no middle");

        } else if (!top && middle && bottom) {
            belt.set(beltForwardTwo);
            System.out.println("middle and bottom");

        } else if (!top && !middle && bottom) {
            belt.set(beltForwardOne);
            System.out.println("only bottom");

        } else if (middle) {
            belt.set(0);
            System.out.println("middle");
        } else if (none) {
            System.out.println("none");
            belt.set(0);
        }

    }

    /**
     * This belt logic looks at the current belt speed to determine the current
     * state. If we are shooting and up to speed, we go forward. Otherwise, we use
     * the following logic. If no balls are detected, we set the speed to 0. If the
     * belt is moving forward, we stop when a ball reaches top. If the belt is
     * moving backward, we stop when a ball reaches middle. If the belt is at rest,
     * we look at the sensors. - top, middleWeak and bottom: no movement - just
     * bottom: move forward until top is triggered (to top) - top and bottom: move
     * backward until middle is triggered (to middle and bottom) - middle and
     * bottom: move forward until top is triggered (to top and middle) This code
     * assumes that the belt will return 0.0 when it is not moving. Also, should we
     * add a timer so that the belt stops in case of errors? Maybe run for 1-2
     * seconds at most?
     */
    public void tempBelt() {
        // BELT STUFF!!!!!!!!!!!!!!!!!
        if (controller.getBButton()) {
            armsSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        if (RobotMap.controller.getTriggerAxis(Hand.kRight) > .6 // if shooter up to speed
                && RobotMap.leftShooterFalcon.getSelectedSensorVelocity() >= Shooter.rpmToRotatPer100Mili(Shooter.rotpm)
                        * Shooter.kEncoderUnitsPerRev) {
            belt.set(beltForwardTwo);
            System.out.println("shooter up to speed");
        } else if (none) {
            System.out.println("none, reset to 0");
            belt.set(0.0);
        } else if (top && middleWeak && bottom) {
            belt.set(0.0);
            System.out.println("top, middleWeak and bottom, reset to/stay at 0");
        } else if (belt.get() == 0.0) {
            // ready for new command
            if (top && bottom) {
                belt.set(beltBackwardsOne);
                System.out.println("top, bottom, go BACK");
            } else if (middleWeak && bottom) {
                belt.set(beltForwardTwo);
                System.out.println("middle weak, bottom, go FORWARD");
            } else if (bottom) {
                belt.set(beltForwardOne);
                System.out.println("bottom, go FORWARD");
            }
        } else if (belt.get() < 0.0) {
            // currently moving forward
            if (top) {
                belt.set(0.0);
                System.out.println("top, STOP forward");
                if (middleWeak) {
                    System.out.println("\t also middle weak");
                }
            }
        } else if (belt.get() > 0.0) {
            // currently moving backward
            if (middle) {
                belt.set(0.0);
                System.out.println("middle, STOP backward");
                if (bottom) {
                    System.out.println("\t also bottom");
                }
            }
        }
    }

    public void toSmartDashboard() {
        SmartDashboard.putNumber("colorSensorHigh Green", colorSensorHigh.getGreen());
        SmartDashboard.putNumber("colorSensorMid Green", colorSensorMid.getGreen());
        SmartDashboard.putNumber("ultrasonic", ultrasonic.getRangeInches());
        SmartDashboard.putBoolean("bottom", bottom);
        SmartDashboard.putBoolean("middle", middle);
        SmartDashboard.putBoolean("top", top);
        SmartDashboard.putBoolean("middle weak", middleWeak);
        SmartDashboard.putBoolean("none", none);
    }

    public void ToggleSolenoids() {
        if (RobotMap.controller.getXButtonPressed()) {

            if (newMattyState == false) {
                armsSolenoid.set(DoubleSolenoid.Value.kForward);
                newMattyState = true;

            } else if (newMattyState == true) {
                armsSolenoid.set(DoubleSolenoid.Value.kReverse);
                newMattyState = false;
            }

        }

    }

}