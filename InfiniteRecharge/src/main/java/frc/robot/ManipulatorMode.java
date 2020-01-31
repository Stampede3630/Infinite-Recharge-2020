/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public interface ManipulatorMode {
    public void intake();
    public void deploy(boolean rocketMode);
    public void engage();
    public void disengage();
    public void intakeAuto();
    public void deployAuto(double robotTime);
    public void endAll();
}
