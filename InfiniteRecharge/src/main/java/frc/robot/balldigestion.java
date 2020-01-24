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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */


public class Balldigestion {

    XboxController controlly;
    WPI_TalonSRX armiedown;//first spin wheel
    WPI_TalonSRX spiceygo; //spikey circle spinny
    WPI_TalonSRX upiddiego; //belt
    //Solenoid newmatty; //lowers the arms
    Timer timmy;
    //ColorSensorV3 s1; // color sensor 1 first spikey
   // ColorSensorV3 s2; //color sensor 2 middle of belt
    ColorSensorV3 s3; // color sensor 3 by the shoot
    Color noColor; 
    Ultrasonic s1;
  Ultrasonic s2;
  
 


public Balldigestion (){
    controlly = new XboxController(0);
    armiedown = new WPI_TalonSRX(9);
    //newmatty = new Solenoid(19); // no solenoid on robo
    upiddiego = new WPI_TalonSRX(11);
    timmy = new Timer(); 
    s3 = new ColorSensorV3(I2C.Port.kOnboard);
    spiceygo = new WPI_TalonSRX(10);
    s1 = new Ultrasonic(8,9);
    s2 = new Ultrasonic(6, 7);
    s1.setAutomaticMode(true);
    s1.setDistanceUnits(Ultrasonic.Unit.kInches);
    s2.setAutomaticMode(true);
    s2.setDistanceUnits(Ultrasonic.Unit.kInches);

}
public void greggory (){
    if(controlly.getAButtonPressed())
    {
        System.out.println("tester");
        timmy.start();//must check
        //newmatty.set(true); 
        armiedown.set(.5);
    }

    if(timmy.hasPeriodPassed(4) && s1.getRangeInches()>3  )
     {
        spiceygo.set(.5);
        System.out.println("called");
        armiedown.set(0);
    }
    
    if (s1.getRangeInches()<3){
        System.out.println(s1.getRangeInches());
        spiceygo.set(0);
        System.out.println("testing");
    }
    
    if(s2.getRangeInches()>3 && s1.getRangeInches()<3){
        upiddiego.set(.5);
    }
        

    if (s2.getRangeInches()<3 && s1.getRangeInches()<3 && s3.getGreen()<700){

        upiddiego.set(.5);
    }
    if(s3.getGreen()<700 && s2.getRangeInches()<3 || s1.getRangeInches()>3 && s2.getRangeInches()<3){

        upiddiego.set(0);

    }
    if (s3.getGreen()>700 && controlly.getBButton()){

    //SHOOT NOW CODE

    }
if (s1.getRangeInches()>3 && s2.getRangeInches()>3 && s3.getGreen()>700){
upiddiego.set(0);


}
SmartDashboard.putNumber("stringy", s3.getRawColor().green);

SmartDashboard.putNumber("proximity", s3.getGreen());

SmartDashboard.putNumber("ultrasonic", s1.getRangeInches());
SmartDashboard.putNumber("ultrasonic", s2.getRangeInches());

}}

    


