/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               *//*----------------------------------------------------------------------------*/


package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class BreakBeam {

    private static BreakBeam breakBeam;
    DigitalInput beam0;
    DigitalInput beam1;
    DigitalInput beam2;
    DigitalInput beam3;
    DigitalInput beam4;
    DigitalInput beam5;



    private BreakBeam() {
        
        beam0 = new DigitalInput(10);
        beam1 = new DigitalInput(11);
        beam2 = new DigitalInput(12);
        beam3 = new DigitalInput(13);
        beam4 = new DigitalInput(18);
        beam5 = new DigitalInput(19);      
    }
    /**
     * @return the beam
     */

    public static BreakBeam getInstance() {
        if (breakBeam == null) {
          breakBeam = new BreakBeam();
        }
        return breakBeam;
      }

    public boolean detectBallHigh(){
        if(beam0.get() && beam1.get() &&beam2.get()){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean detectBallMid(){
        if(!beam3.get() && beam4.get() && beam5.get()){
            return true;
        }
        else{
            return false;
        }
    }

    public void toSmartDashBoard()
    {
        SmartDashboard.putBoolean("beam0", beam0.get());
        SmartDashboard.putBoolean("beam1", beam1.get());
        SmartDashboard.putBoolean("beam2", beam2.get());
        SmartDashboard.putBoolean("beam3", beam3.get());
        SmartDashboard.putBoolean("beam4", beam4.get());
        SmartDashboard.putBoolean("beam5", beam5.get());

    }


}
