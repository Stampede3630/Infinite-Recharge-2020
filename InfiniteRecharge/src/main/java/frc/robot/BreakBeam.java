/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               *//*----------------------------------------------------------------------------*/


package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;



public class BreakBeam {

    private DigitalOutput beam[];
    
    public BreakBeam() {

        beam[0] = new DigitalOutput(0);
        beam[1] = new DigitalOutput(1);
        beam[2] = new DigitalOutput(2);
        beam[3] = new DigitalOutput(3);
        beam[4] = new DigitalOutput(4);
        beam[5] = new DigitalOutput(5);
        beam[6] = new DigitalOutput(6);
        beam[7] = new DigitalOutput(7);
        beam[8] = new DigitalOutput(8);
        beam[9] = new DigitalOutput(9);

    }
    /**
     * @return the beam
     */
    public DigitalOutput[] getBeam() {
        return beam;
    }

    public boolean detectBall(int top, int mid, int low){
        if(beam[top].get() && beam[mid].get() &&beam[low].get()){
            return true;
        }
        else{
            return false;
        }
    }


}
