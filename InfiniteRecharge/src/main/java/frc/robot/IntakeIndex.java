/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */

public class IntakeIndex {

    XboxController controller;
    WPI_TalonSRX intakeWheels;// first spin wheel
    WPI_TalonSRX pinwheel; // from box to belt
    WPI_TalonSRX belt; 
    DoubleSolenoid newmatty; //lowers the arms
    Timer timmy;
    Ultrasonic ultrasonic; //on the ground of the belt box
    ColorSensorV3 colorSensorLow;
    ColorSensorV3 colorSensorHigh; 
    Color noColor;

    double threshold = 20000;

    public IntakeIndex() {
    controller = new XboxController(0);
    intakeWheels = new WPI_TalonSRX(9);
    newmatty = new DoubleSolenoid(19, 5); // 2 solenoid on r
    belt = new WPI_TalonSRX(11);
    timmy = new Timer(); 
    colorSensorHigh = new ColorSensorV3(I2C.Port.kMXP);
    pinwheel = new WPI_TalonSRX(10);
    ultrasonic = new Ultrasonic(5,3);
    colorSensorLow = new ColorSensorV3(I2C.Port.kOnboard);
    ultrasonic.setAutomaticMode(true);
    ultrasonic.setDistanceUnits(Ultrasonic.Unit.kInches);
    //colorSensorLow.setAutomaticMode(true);
    //colorSensorLow.setDistanceUnits(Ultrasonic.Unit.kInches);
}
    
//most recent intake machine
public void index (){
    System.out.println(pinwheel.get());

    if(controller.getAButton())
    {
        System.out.println("tester");
        //timer.stop();
        timmy.reset();
        timmy.start();
        newmatty.set(true); 
        intakeWheels.set(.5);
        
    }
    else {
        intakeWheels.set(0);
        System.out.print(timmy.get());
        
    }
    //if its been 1.5 sec or there's something in the bottom
    if(timmy.get() > 1.5 || ultrasonic.getRangeInches()>200 || ultrasonic.getRangeInches() < 3 || timmy.get() == 0)  
     {
        pinwheel.set(0);      
    }
    else{
        pinwheel.set(-.8);
    }
    //if something is in the top
    if (colorSensorHigh.getGreen() > threshold)
    {
        belt.set(0);
    }
    //if a ball is in the bottom
    else if(colorSensorLow.getGreen()<threshold && (ultrasonic.getRangeInches()>200 || ultrasonic.getRangeInches() < 8)){
        belt.set(-.5);
    }
    //if something is in the middle and the bottom
    else if (colorSensorLow.getGreen()>threshold && (ultrasonic.getRangeInches()>200 || ultrasonic.getRangeInches() < 8)  && colorSensorHigh.getGreen()<threshold){
        belt.set(-.5);
    }
    else
    {
        belt.set(0);
    }
   
   // if (colorSensorHigh.getGreen()>threshold && controller.getBButton()){

    //SHOOT NOW CODE
    if (controller.getBButton()){
        newmatty.set(false);
    }
    
    }
    
    /*
        SmartDashboard.putNumber("stringy", colorSensorHigh.getRawColor().green);
        SmartDashboard.putNumber("proximity", colorSensorHigh.getGreen());
        SmartDashboard.putNumber("ultrasonic", ultrasonic.getRangeInches());
        SmartDashboard.putNumber("ultrasonic", colorSensorLow.getGreen());
    */



    public void toSmartDashboard(){
        SmartDashboard.putNumber("colorSensorHigh Green", colorSensorHigh.getGreen());
        SmartDashboard.putNumber("colorSensorLow Green", colorSensorLow.getGreen());
        SmartDashboard.putNumber("ultrasonic", ultrasonic.getRangeInches());
    }

}
