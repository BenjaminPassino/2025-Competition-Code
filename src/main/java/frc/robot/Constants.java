// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  //ALGAE
  public static final int LeftAlgaeMotorPort = 0; //TODO This will need updated
  public static final int RightAlgaeMotorPort = 0;
  //CORAL
  public static final int CoralScoringMotorPort = 0;
  public static final int CoralArmMotorPort = 0;
  //ELEVATOR
  public static final int ElevatorMotorPort = 0;
  //DEEP CLIMB
  public static final int DeepClimbMotorPort = 0;
  
  //Limit Switch
  public static final int CoralArmLimitSwitchPort = 0;
  public static final int ElevatorBottomLimitSwitchPort = 0;
  public static final int DeepClimbArmLimitSwitchPort = 0;
  public static final int DeepClimbCageLimitSwitchPort = 0;
  //end of mech items


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    
  }
}
