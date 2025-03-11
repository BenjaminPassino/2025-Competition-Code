// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /* MECH ITEMS */
//LeftAlgaeMotor
//RightAlgaeMotor
//CoralScoringMotor
//CoralArmMotor 
//ElevatorMotor
//DeepClimbMotor
//ElevatorBottomLimitSwitch
//DeepClimbArmLimitSwitch
//DeepClimbCageLimitSwitch
//CoralArmLimitSwitch
//CoralArmEncoder

  //Motor ID's

  //ALGAE SPARK
  public static final int LeftAlgaeMotorPort = 0;  //20
  public static final int RightAlgaeMotorPort = 1; //21
  //CORAL SPARK
  public static final int CoralScoringMotorPort = 3; //22
  public static final int CoralArmMotorPort = 23; //CAN
  //ELEVATOR SPARK
  public static final int ElevatorMotorPort = 24; //CAN
  //DEEP CLIMB SPARK
  public static final int DeepClimbMotorPort = 25; //CAN
  //DEEP CLIMB SERVO
  public static final int DeepClimbServoPort = 4;
  
  //Limit Switch
  public static final int CoralArmLimitSwitchPort = 1;
  public static final int ElevatorBottomLimitSwitchPort = 2;
 // public static final int DeepClimbArmLimitSwitchPort = 3;
  public static final int DeepClimbCageLimitSwitchPort = 0;
  //end of mech items

  /*    Motor Speeds     */


  //REEF
 public static double CoralScoringSpeed = 0.5;
 public static double CoralArmSpeed = 0.5;
 public static double CoralCollectionSpeed = -0.5;
 public static double CoralCollectionArmSpeed = -0.5;
 public static double AlgaeCollectionSpeed = -0.5;
 public static double AlgaeScoringSpeed = 0.5;
 public static double ElevatorUpSpeed = 0.5;
 public static double ElevatorDownSpeed = -0.5;

 public static double L1Height = -33.8;
 public static double L2Height = -64.7;
 public static double L3Height = -119.5;
 public static double L4Height = -193.6;
 public static double CoralStationHeight= -33.8; //set to l1 height
 public static double L1Angle = -11.1;
 public static double L2Angle = -11.1;
 public static double L3Angle = -11.1;
 public static double L4Angle = -10.2;
 public static double ZeroedAngle = 0;
 public static double CoralStationAngle= -4.5; //set to l4 angle

 public static double CoralMaximum=1;
 public static double CoralMinimum=0;


 //PID
 public static double Pvar = 0.02;  //TODO Adjust speed to be faster in matches
 public static double Ivar = 0;
 public static double Dvar = 0; 
 public static double TARGETPOSITION=1;

 public static double wPvar = 0.028;
 public static double wIvar = 0;
 public static double wDvar = 0; 

 public static int cPvar = 1;
 public static int cIvar = 1;
 public static int cDvar = 1; 

 //ENCODER

 public static double Distance = 1;
 //public static double ElevatorHeight = 0;
 public static double VVristPosition = 0;


 //AUTO
 public static int AutoTimerValue = 0;
 public static double EndRelease = 0.2;
 public static double L4TimeAuto = 1.5;
 public static double AutoCollection = 1;
 public static double StationToReef = 1;


  //CLIMB
  public static double ReleasePortSpeed = -0.1;


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
  }


          





            
}
