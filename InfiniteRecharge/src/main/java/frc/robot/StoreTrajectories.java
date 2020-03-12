/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/**
 * Add your docs here.
 */
public class StoreTrajectories {
    
static Trajectory path1;
static Trajectory path2;
/*
public static Trajectory path3 =  TrajectoryGenerator.generateTrajectory(new Pose2d (2.4,3.05, new Rotation2d(0)),
List.of(
    new Translation2d(2.4,6.166)),//Trench ball 1
    //new Translation2d(.704,7.08)),//Trench ball 2
    //new Translation2d(.704, 7.99)),//Trench ball 3
    new Pose2d(2.4,7.99, new Rotation2d(0)), TrajectoryContainer.config); //intake logic path 2
    */
static Trajectory  path4; //intake logic path 3
static Trajectory path5; //intake logic path 4
static Trajectory path6; //intake logic path 5
static Trajectory path7; //intake logic path 6, changed
static Trajectory path8; //intake path starting at wall
static Trajectory path9; //path with angles

   static {
       /*
         path1 = TrajectoryGenerator.generateTrajectory(new Pose2d(2.40, 3.05, new Rotation2d(0)),
         List.of(
            new Translation2d(.704,6.166),//Trench ball 1
            new Translation2d(.704,7.08),//Trench ball 2
            new Translation2d(.704, 7.99),//Trench ball 3
            new Translation2d(2.907, 6.359),//Center ball 1
            new Translation2d(3.068, 5.97 ),//Center ball 2
            new Translation2d(0.947,6.166 )),//Avoid hitting pole
        new Pose2d(2.40,3.05, new Rotation2d(0)), TrajectoryContainer.config);
         //Math.toRadians(angdeg);
 
        path2= TrajectoryGenerator.generateTrajectory( new Pose2d (2.4,3.05, new Rotation2d(0)),
        List.of(
            new Translation2d (3.69,4.32),// avoid hitting pole 1
            new Translation2d (3.69,5.79),//ball 3 center
            new Translation2d (4.08,5.95), //center ball 4
            new Translation2d (4.46,6.11),//center ball 5
            new Translation2d (4.46,4.9), //AVOID HITTING POLE, this is a guess
            new Translation2d (6.23,6.36),// trench 2 ball 6
            new Translation2d (7.74,6.36)),//last ball or trench 2 ball 7
        new Pose2d(2.40,3.05, new Rotation2d(0)), TrajectoryContainer.config);//end where we started //re do this move it back a little
       //path/trajectory 2
       path3 = TrajectoryGenerator.generateTrajectory(new Pose2d (2.4,3.05, new Rotation2d(0)),
        List.of(
            new Translation2d(2.4,6.166)),//Trench ball 1
            //new Translation2d(.704,7.08)),//Trench ball 2
            //new Translation2d(.704, 7.99)),//Trench ball 3
            new Pose2d(2.4,7.99, new Rotation2d(0)), TrajectoryContainer.config);
            //new Pose2d(2.40,3.05, new Rotation2d(0)), TrajectoryContainer.config);
    //start toward right wall, shoot 3, pick up 3 trench and shoot (path2 intake logic)
       
       path4= TrajectoryGenerator.generateTrajectory(new Pose2d (2.4,3.05, new Rotation2d(0)),
       List.of(
            new Translation2d(.704,6.166),//Trench ball 1
            new Translation2d(.704,7.08),//Trench ball 2
            new Translation2d(.704, 7.99),//Trench ball 3
            new Translation2d(2.907, 6.359),//Center ball 1
            new Translation2d(3.068, 5.97 ),//Center ball 2
            new Translation2d(0.947,6.166 )),//Avoid hitting pole
       new Pose2d(2.40,3.05, new Rotation2d(0)), TrajectoryContainer.config);
       //start toward right wall, shoot 3, pick up 3 trench and 2 shield gemerator and shoot close trench (path3 intake logic)
       
       path5 = TrajectoryGenerator.generateTrajectory(new Pose2d (5.81,3.05, new Rotation2d(0)),
        List.of(
        new Translation2d (6.23,6.36),// trench 2 ball 6
        new Translation2d (7.74,6.36)),//last ball or trench 2 ball 7
    new Pose2d(2.40,3.05, new Rotation2d(0)), TrajectoryContainer.config);
    //start toward left wall, intake 2 trench, shoot close to trench (path4 intake logic)
    // THIS PATH DOESN'T REALLY DO ANYTHING KINDA USELESS
        path6 = TrajectoryGenerator.generateTrajectory(
        new Pose2d (5.81,3.05, new Rotation2d(0)),
                null, new Pose2d(2.40, 3.05, new Rotation2d(0)), TrajectoryContainer.config);
    //start loading station, drive middle, and shoot (path5 intake logic)
    // ALSO USELESS
        path7 = TrajectoryGenerator.generateTrajectory(new Pose2d (5.81,3.05, new Rotation2d(0)),
        List.of(
        new Translation2d (2.4, 3.05),
        new Translation2d (3.69,4.32),// avoid hitting pole 1
        new Translation2d (3.69,5.79),//ball 3 center
        new Translation2d (4.08,5.95), //center ball 4
        new Translation2d (4.46,6.11),//center ball 5
        new Translation2d (4.46,4.9), //AVOID HITTING POLE, this is a guess
        new Translation2d (6.23,6.36),// trench 2 ball 6
        new Translation2d (7.74,6.36)),//last ball or trench 2 ball 7  
     new Pose2d(2.40,3.05, new Rotation2d(0)), TrajectoryContainer.config);
     //same as path 2 just starting at loading station
         path8 = TrajectoryGenerator.generateTrajectory(new Pose2d (0,3.05, new Rotation2d(0)),
     List.of(
     new Translation2d (1.41,5.25),// angle shooting point
     new Translation2d(.704,6.166),//Trench ball 1
     new Translation2d(.704,7.08),//Trench ball 2
     new Translation2d(.704, 7.99),//Trench ball 3
     new Translation2d(2.907, 6.359),//Center ball 1
     new Translation2d(3.068, 5.97 )),//Center ball 2
  new Pose2d(1.41,5.25, new Rotation2d(0)), TrajectoryContainer.config);
    //path starting at wall with angle shooting point
        
*/
}}
