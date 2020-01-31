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
public class Constants 
{

    public static final double strafeKP = 0.115;//BL=1000
    public static final double strafeKI = 0;
    public static final double strafeKD = 0;
    public static final double strafeOutput = 0.6;
    public static final double strafeTolerance = 2;
    
    public static final double forwardKP = 0.1;
    public static final double forwardKI = 0;
    public static final double forwardKD = 0;
    public static final double forwardOutput = 0.6;
    public static final double forwardTolerance = 2;
    public static final double fullTargetTa = 100;

    public static final double turnKP = 0.03;
    public static final double turnKI = 0;
    public static final double turnKD = 0;
    public static final double turnOutput = 1;
    public static final double turnTolerance = 2;

    public static final double pidLowSpeed = 0.1;


    public static final double h1 = 45.75; //distance from bottom of robot to camera (inches)
    public static final double h2 = 31.50; //distance from bottom of board to far left target (inches)
    public static final double h3 = 4; // distance from bottom of robot to bottom limelight (inches)
    public static final double w1 = 13.78; //half of robot distance in x direction (inches)
    public static final double alphaXOne = 0;
    public static final double alphaYOne = 0;

    public static final int timeOutMs = 10;

    public final static int aButton = 1;
	public final static int bButton = 2;
	public final static int xButton = 3;
	public final static int yButton = 4;
	public final static int leftBumper = 5;
	public final static int rightBumper = 6;
	public final static int backButton = 7;
	public final static int startButton = 8;
	public final static int lStickButton = 9;
	public final static int rStickButton = 10;
	public final static int lStickXAxis = 0;
	public final static int lStickYAxis = 1;
	public final static int lTriggerAxis = 2;
	public final static int rTriggerAxis = 3;
	public final static int rStickXAxis = 4;
	public final static int rStickYAxis = 5;
    public final static double deadzone = 0.2;
    
//    public final static double multiplierScaleUp = 1;
    public final static double normalSpeed = 0.7;
    public final static double rocketBallLaunchUpSpeed = 0.8;
    public final static double rocketBallLaunchDownSpeed = -0.8;

    public static final double closestIntake = 0;
    public static final double leftmost = 1;
    public static final double rightmost = 2;
    public static final double closestDeploy = 3;
    public static final double hatchDriving = 4;
    public static final double defaultPL = 6;
    public static final String hatchLimelight = "limelight-two";
    public static final String ballLimelight = "limelight-one";
    public static final double deployLong = 0;
    public static final double deployTY = 0;

    public static boolean autoDeploy = false;
    public static boolean lostTarget = false;
    public static boolean ballTop = false;
    public static boolean ballBottom = false;
    public static String limelight = hatchLimelight;
    public static String pipeline = "deploy";
    public static double pipelineNumber = 0;
    public static double robotAngle = 0;
    public static boolean toRocket = false;
    public static boolean ballFollowerOn = false;
    public static boolean ballFollowerExecuting = false;
    public static boolean ballManipulator = false;
    public static String forwardFromWidget = "";
    public static double tx = 0;
    public static double ty = 0;
    public static double ta = 0;
    public static double tv = 0;
    public static double ts = 0;
}
