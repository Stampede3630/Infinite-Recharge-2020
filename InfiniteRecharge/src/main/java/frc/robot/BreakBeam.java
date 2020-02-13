/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               *//*----------------------------------------------------------------------------*/


package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;



public class BreakBeam {

    private DigitalOutput beam[];
    public static BreakBeam breakBeam;
    
    private BreakBeam() {
        beam[0] = new DigitalOutput(10);
        beam[1] = new DigitalOutput(11);
        beam[2] = new DigitalOutput(12);
        beam[3] = new DigitalOutput(13);
        beam[4] = new DigitalOutput(14);
        beam[5] = new DigitalOutput(15);
        
    }
    /**
     * @return the beam
     */
    public DigitalOutput[] getBeam() {
        return beam;
    }

    public static BreakBeam getInstance() {
        if (breakBeam == null) {
          breakBeam = new BreakBeam();
        }
        return breakBeam;
      }

    public boolean detectBallHigh(){
        if(beam[10].get() && beam[11].get() &&beam[12].get()){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean detectBallMid(){
        if(!beam[13].get() && beam[14].get() &&beam[15].get()){
            return true;
        }
        else{
            return false;
        }
    }


}
