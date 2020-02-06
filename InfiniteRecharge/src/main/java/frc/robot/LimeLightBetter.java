/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class LimeLightBetter {
    /*
    
    Double tx;
    Double ty;
    Double ts;
    Double ta;
    Double tv;

    NetworkTable table;
    NetworkTableInstance tableInstance;
    boolean validTarget;
    double driveCommand;
    double steerCommand;

    double KpAim = 0.05;
    double KpDistance = 0.1;
    double min_aim_command = 0.05;
    
    WPI_TalonSRX motorFL;
    WPI_TalonSRX motorFR;
    WPI_TalonSRX motorRL;
    WPI_TalonSRX motorRR;

    MecanumDrive mecanum;
    Double distanceAdjust;

    AHRS ahrs;
    Encoder encoder;
    PIDController pidTurn;
    PIDController pidStrafe;
    PIDController pidStraight;

    DriveTrain driveTrain;

    public LimeLightBetter(){
        
        driveTrain = new DriveTrain();

        table = NetworkTableInstance.getDefault().getTable("limelight");
        
          motorFL = new WPI_TalonSRX(4);
          motorFR = new WPI_TalonSRX(9);
          motorRL = new WPI_TalonSRX(2);
          motorRR = new WPI_TalonSRX(1);
          mecanum = new MecanumDrive(motorFL, motorFR, motorRL, motorRR);

          tableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);

        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();
        encoder = new Encoder(1, 0); //get sources
        pidTurn = new PIDController(.15, 0, 0);
        pidStrafe = new PIDController(.1, 0, 0);
        pidStraight = new PIDController(.1, 0, 0); //Straight = forward/backward

    }

    public void driveStraight() {
        double error = -ahrs.getRate();
        double turning = pidTurn.getP() * error;

        mecanum.driveCartesian(.3, 0, turning);
        // System.out.println(.3 + pidControllerStraight.getP() * error)

    }

    public void lineUpDist() {
        double error = -ahrs.getRate();
        double turning = pidTurn.getP() * error;
        driveTrain.meecanumDrive.driveCartesian(pidStraight.calculate(measurement, setpoint), pidStrafe.calculate(measurement, setpoint), turning);
        
    }



    public void limelightAdjust(){

        tx = table.getEntry("tx").getDouble(0); //horizontal offset from crosshair to target
        ty = table.getEntry("ty").getDouble(0); //vertical offset ^^
        ta = table.getEntry("ta").getDouble(0); //target area (0% to 100%)
        tv = table.getEntry("tv").getDouble(0); //if there's valid targets

        double distance = 40/(Math.tan(ty+15)); //camera height, camera angle

            double heading_error = -tx;
            double distance_error = .30-ta; //10% of screen - target area
            double steering_adjust = 0.0;
            double min_aim_command = 0.05; //minimum motor power

            if (tx > 1.0)
            {
                    steering_adjust = KpAim*heading_error - min_aim_command;
            }
            else if (tx < 1.0)
            {
                    steering_adjust = KpAim*heading_error + min_aim_command;
                    return;
            }

            distanceAdjust = KpDistance * distance_error;
            
            System.out.println("tx = " + tx);
            System.out.println("ty = " + ty);
            System.out.println("distance adjust = " + distanceAdjust);
            System.out.println("steering adjust = " + steering_adjust);

            mecanum.driveCartesian(distanceAdjust, heading_error, steering_adjust); //y, x, rotation


    }
    */
}
