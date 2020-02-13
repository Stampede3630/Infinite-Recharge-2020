/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Add your docs here.
 */
public class Chooser {
    public SendableChooser<Boolean> climbChooser;

    public Chooser(){
        climbChooser = new SendableChooser<Boolean>();
        climbChooser.setDefaultOption("angles", true);
        climbChooser.addOption("climb", false);
    }

    public void updateChooser(){
        if(climbChooser.getSelected()){
            //run angle turn, hex line up

        }
        else{
            //run climb d-pad, climb angle
        }
    }

}
