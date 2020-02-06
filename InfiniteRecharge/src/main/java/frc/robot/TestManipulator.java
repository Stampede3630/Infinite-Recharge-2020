/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Add your docs here.
 */
public class TestManipulator {
    Counter counter = new Counter(4);
    WPI_TalonSRX intake = new WPI_TalonSRX(9);
    WPI_TalonSRX belt = new WPI_TalonSRX(11);
    WPI_TalonSRX highRoller = new WPI_TalonSRX(10);
    private boolean solenoid1Forward = false;
    private boolean solenoid2Forward = false;

   
    public TestManipulator()
    {
        belt.setNeutralMode(NeutralMode.Brake);
        counter.setSemiPeriodMode(true);
    }
    public void periodic()
    {
        if(RobotMap.controller.getAButton())
        {
            intake.set(.75);
            highRoller.set(-.4);
        }
        else{
            intake.set(0);
            highRoller.set(0);
        }


        if(RobotMap.controller.getTriggerAxis(Hand.kRight)>0.5)
        {
            belt.set(-.7);
        }
        else if(RobotMap.controller.getBumper(Hand.kRight)){
            belt.set(0.7);
        }
        else {
            belt.set(0);
        }

//SOLENOID ZONE

        if(RobotMap.controller.getXButtonPressed())
        {
            if(solenoid1Forward==false){
                RobotMap.solenoid1.set(Value.kForward);
                solenoid1Forward = true;
            }

            else if(solenoid1Forward==true){
                RobotMap.solenoid1.set(Value.kReverse);
                solenoid1Forward = false;

            }
        }


        if(RobotMap.controller.getYButtonPressed()){
            if(solenoid2Forward==false){
                RobotMap.solenoid1.set(Value.kForward);
                solenoid2Forward = true;

            }
    
            else if(solenoid2Forward==true){
                RobotMap.solenoid1.set(Value.kReverse);
                solenoid2Forward = false;

            }
        }
        
        /*if(RobotMap.controller.getXButton())
        {
            highRoller.set(-.8);
        }
        else
        {   
            highRoller.set(0);
        }*/
    }
    public void getUltra(){

        SmartDashboard.putNumber("Ultrasonic Period", counter.getPeriod());

    }
}
