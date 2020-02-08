/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    DoubleSolenoid newmatty; // lowers the arms
    Timer timmy;
    Ultrasonic ultrasonic; // on the ground of the belt box
    ColorSensorV3 colorSensorLow;
    ColorSensorV3 colorSensorHigh;
    Color noColor;
    Shoot shoot;
    double threshold;
    boolean bottom, middle, top, none;
    private boolean newMattyState = false;

    public IntakeIndex() {
        controller = new XboxController(0);
        intakeWheels = new WPI_TalonSRX(9);
        newmatty = new DoubleSolenoid(2, 3); // 2 solenoid on r
        belt = new WPI_TalonSRX(11);
        timmy = new Timer();
        colorSensorHigh = new ColorSensorV3(I2C.Port.kMXP);
        pinwheel = new WPI_TalonSRX(10);
        ultrasonic = new Ultrasonic(5, 3);
        colorSensorLow = new ColorSensorV3(I2C.Port.kOnboard);

        ultrasonic.setAutomaticMode(true);
        ultrasonic.setDistanceUnits(Ultrasonic.Unit.kInches);
        // colorSensorLow.setAutomaticMode(true);
        // colorSensorLow.setDistanceUnits(Ultrasonic.Unit.kInches);

        intakeChooser = new SendableChooser<Integer>();
        intakeChooser.addDefault("Automatic", 1);
        intakeChooser.addObject("Manual", 0);
    }

    public void updateBooleans() {
        threshold = 13000;

        bottom = false;
        middle = false;
        top = false;
        none = true;

        if (ultrasonic.getRangeInches() > 200 || ultrasonic.getRangeInches() < 9) {
            bottom = true;
            none = false;
        }
        if (colorSensorHigh.getGreen() > threshold) {
            top = true;
            none = false;
        }
        if (colorSensorLow.getGreen() > threshold) {
            middle = true;
            none = false;
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

    public void index() {  // most recent intake machine
        System.out.println(pinwheel.get());
        updateBooleans();

        if (controller.getAButton()) {
            System.out.println("tester");
            timmy.reset();
            timmy.start();
            intakeWheels.set(.5);
        }

        else {
            intakeWheels.set(0);
            System.out.print(timmy.get());
        }
        
        if (timmy.get() > 1.5 || bottom == true || timmy.get() == 0) { // if its been 1.5 sec or there's something in the bottom
            pinwheel.set(0);
        }
        
        else {
            pinwheel.set(-.8);
        }

        // BELT STUFF!!!!!!!!!!!!!!!!!

        if (middle && top && bottom
                && RobotMap.shooter1.getSelectedSensorVelocity() < Constants.rpmToRotatPer100Mili(Shoot.rotpm)
                        * Constants.kEncoderUnitsPerRev) {
            belt.set(0);
        }

        if (controller.getBButton()) {
            newmatty.set(DoubleSolenoid.Value.kReverse);
        }

        else { 
            if (!middle && bottom) {// if a ball is in the bottom
                belt.set(-.5);
            }

             else if (middle && bottom && !top) {// if something is in the middle and the bottom
                belt.set(-.5);
            }

            else if (!middle && top) { // if something is in top but not middle
                belt.set(.5);
            }

            else if (RobotMap.controller.getTriggerAxis(Hand.kRight) > .6 // if shooter up to speed
                    && RobotMap.shooter1.getSelectedSensorVelocity() >= Constants.rpmToRotatPer100Mili(Shoot.rotpm)* Constants.kEncoderUnitsPerRev) {
                belt.set(-.5);
            } else {
                belt.set(0);
            }
        }

    }

    public void toSmartDashboard() {
        SmartDashboard.putNumber("colorSensorHigh Green", colorSensorHigh.getGreen());
        SmartDashboard.putNumber("colorSensorLow Green", colorSensorLow.getGreen());
        SmartDashboard.putNumber("ultrasonic", ultrasonic.getRangeInches());
    }

    public void ToggleSolenoids() {
        if (RobotMap.controller.getXButtonPressed()) {

            if (newMattyState == false) {
                newmatty.set(DoubleSolenoid.Value.kForward);
                newMattyState = true;

            } else if (newMattyState == true) {
                newmatty.set(DoubleSolenoid.Value.kReverse);
                newMattyState = false;
            }

        }

    }

}