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

	private static BreakBeam instance;

	static {
		instance = new BreakBeam();
	}

	public static BreakBeam getInstance() {
		return instance;
	}

	// TODO: Move to RobotMap
	private DigitalInput beam0;
	private DigitalInput beam1;
	private DigitalInput beam2;
	private DigitalInput beam3;
	private DigitalInput beam4;
	private DigitalInput beam5;
	private DigitalInput beam6;
	private DigitalInput beam7;

	private BreakBeam() {

		beam0 = new DigitalInput(10);
		beam1 = new DigitalInput(11);
		beam2 = new DigitalInput(12);
		beam3 = new DigitalInput(13);
		beam4 = new DigitalInput(18);
		beam5 = new DigitalInput(19);
		beam6 = new DigitalInput(20);
		beam7 = new DigitalInput(21);
	}

	public boolean detectBallHigh() {
		if (!beam0.get() && !beam1.get()/* && beam2.get()*/) {
			return true;
		} else {
			return false;
		}
	}

	public boolean detectBallMid() {
		if (/*beam2.get() && */!beam3.get() && !beam4.get() /*&& beam0.get() && beam1.get() Hi andy was here*/ /*&& beam5.get()*/) {
			return true;
		} else {
			return false;
		}
	}

	public boolean detectBallLow() {
		if (/*beam5.get() &&*/ !beam6.get() && !beam7.get()) {
			return true;
		} else {
			return false;
		}
    }
    public boolean noBalls(){
        if(beam0.get() && beam1.get() && beam3.get() && beam4.get() && beam6.get() && beam7.get()){
            return true;
        }
        else{
            return false;
        }
	}
	public boolean detectLimbo(){
		if(beam1.get() && beam3.get() && beam4.get() && beam6.get()){
			return true;
		}
		else{
			return false;
		}
	}

	public boolean detectWeakBottom()
	{
		if (/*beam5.get() &&*/ !beam6.get() || !beam7.get()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean detectWeakTop()
	{
		if (!beam0.get() || !beam1.get()/* && beam2.get()*/) {
			return true;
		} else {
			return false;
		}
	}

	public boolean detectWeakMiddle()
	{
		if (/*beam2.get() && */!beam3.get() || !beam4.get() /*&& beam0.get() && beam1.get() Hi andy was here*/ /*&& beam5.get()*/) {
			return true;
		} else {
			return false;
		}
	}

	public boolean getBottomGap()
	{
		if(!beam5.get())
		{
			return true;
		}
		else
		{
			return false;

		}
	}

	public boolean getVeryBottom()
	{
		if(!beam7.get())
		{
			return true;
		}
		else
		{
			return false;

		}

	}
	
	public boolean bottomToMiddle()
	{
		if((!beam7.get() || !beam6.get() || !beam5.get() || !beam4.get()) && beam3.get())
		{
			return true;
		}
		else
		{
			return false;
		}
	} 

	public boolean bottomMiddleToTop()
	{
		if( (!beam7.get() || !beam6.get()) && (!beam4.get() || !beam3.get() || !beam2.get() || !beam1.get()) && beam0.get())
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	public boolean middleTopRefine()
	{
		if( !beam1.get() && beam0.get())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	public void toSmartDashBoard() {
		SmartDashboard.putBoolean("beam0 TT", beam0.get());
		SmartDashboard.putBoolean("beam1 TB", beam1.get());
		SmartDashboard.putBoolean("beam2 GAP", beam2.get());
		SmartDashboard.putBoolean("beam3 MT", beam3.get());
		SmartDashboard.putBoolean("beam4 MB", beam4.get());
        SmartDashboard.putBoolean("beam5 GAP", beam5.get());
        SmartDashboard.putBoolean("beam6 BT", beam6.get());
        SmartDashboard.putBoolean("beam7 BB", beam7.get());
	}
}
