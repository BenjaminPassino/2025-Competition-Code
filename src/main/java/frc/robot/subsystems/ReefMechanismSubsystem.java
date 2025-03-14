package frc.robot.subsystems;

//  import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
//  import edu.wpi.first.math.controller.PIDController;
//  import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj.DigitalInput;
//  import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


import frc.robot.Constants;

public class ReefMechanismSubsystem extends SubsystemBase{
    /* Motors */
    //algae motors
    private final static PWMSparkMax LeftAlgaeMotor = new PWMSparkMax(Constants.LeftAlgaeMotorPort);
    private final static PWMSparkMax RightAlgaeMotor = new PWMSparkMax(Constants.RightAlgaeMotorPort);
    //Coral motors
    private final static PWMSparkMax CoralScoringMotor = new PWMSparkMax(Constants.CoralScoringMotorPort);
    private final static SparkMax CoralArmMotor = new SparkMax(Constants.CoralArmMotorPort,MotorType.kBrushless);
    //Elevator motor
    private final static SparkMax ElevatorMotor = new SparkMax(Constants.ElevatorMotorPort,MotorType.kBrushless);
   // private final static FeedbackSensor ElevatorFeedback = new FeedbackSensor(ElevatorMotor.getAbsoluteEncoder());
 private RelativeEncoder ElevatorEncoder = ElevatorMotor.getEncoder();
 private RelativeEncoder CoralArmEncoder = CoralArmMotor.getEncoder();
    
        /* Limit Switch */
       
// that feeling when you and your friends take a boat to Istanbul and eat bacon on top of a mountain bc you were bored in history class
 
        // private final DigitalInput CoralArmLimitSwitch = new DigitalInput(Constants.CoralArmLimitSwitchPort);
    
        /* Encoders */
        //private final Encoder CoralArmEncoder = new Encoder(5, 6); //
       // private final Encoder ElevatorEncoder = new Encoder(7,8);
        private double EncoderDistance =1;
        private SparkMaxConfig motorconfig = new SparkMaxConfig();
        private SparkMaxConfig VVristMotorconfig = new SparkMaxConfig();
        private SparkClosedLoopController ElevatorPID = ElevatorMotor.getClosedLoopController();
        private SparkClosedLoopController VVristPID = CoralArmMotor.getClosedLoopController();


        
    

        
            
               // public Command mechanismInitializeCommand() {
                    // Inline construction of command goes here.
                    // Subsystem::RunOnce implicitly requires `this` subsystem.
                    
              //    }
            
              public Command Setup(){
                return runOnce(
                () -> {
                
                  System.out.println("i also work too yay :)");
                SmartDashboard.putNumber("CoralScoringSpeed",Constants.CoralScoringSpeed);  //Puts values to smart dashboard
                SmartDashboard.putNumber("CoralArmSpeed",Constants.CoralArmSpeed);
                SmartDashboard.putNumber("CoralCollectionSpeed",Constants.CoralCollectionSpeed);
                SmartDashboard.putNumber("CoralCollectionArmSpeed",Constants.CoralCollectionArmSpeed);
                SmartDashboard.putNumber("AlgaeCollectionSpeed",Constants.AlgaeCollectionSpeed);
                SmartDashboard.putNumber("AlgaeScoringSpeed",Constants.AlgaeScoringSpeed);
                SmartDashboard.putNumber("ElevatorUpSpeed",Constants.ElevatorUpSpeed);
                SmartDashboard.putNumber("ElevatorDownSpeed",Constants.ElevatorDownSpeed);
              
                SmartDashboard.putNumber("L1",Constants.L1Height);
                SmartDashboard.putNumber("L2",Constants.L2Height);
                SmartDashboard.putNumber("L3",Constants.L3Height);
                SmartDashboard.putNumber("L4",Constants.L4Height);
              
                //SmartDashboard.putNumber("ElevatorEncoder",);
                SmartDashboard.putNumber("VVristPosition",Constants.VVristPosition);
                SmartDashboard.putNumber("ElevatorP",Constants.Pvar);
                SmartDashboard.putNumber("ElevatorI",Constants.Ivar);
                SmartDashboard.putNumber("ElevatorD",Constants.Dvar);
                SmartDashboard.putNumber("VVristP",Constants.wPvar);
                SmartDashboard.putNumber("VVristI",Constants.wIvar);
                SmartDashboard.putNumber("VVristD",Constants.wDvar);
              
              }
                );
              }

public Command Updater(){ //updates smart dashboard values
  return runOnce(
() -> {
  Constants.CoralScoringSpeed = (SmartDashboard.getNumber("CoralScoringSpeed",Constants.CoralScoringSpeed));
  Constants.CoralArmSpeed = (SmartDashboard.getNumber("CoralArmSpeed",Constants.CoralArmSpeed));
  Constants.CoralCollectionSpeed = (SmartDashboard.getNumber("CoralCollectionSpeed",Constants.CoralCollectionSpeed));
  Constants.CoralCollectionArmSpeed = (SmartDashboard.getNumber("CoralCollectionArmSpeed",Constants.CoralCollectionArmSpeed));
  Constants.AlgaeCollectionSpeed = (SmartDashboard.getNumber("AlgaeCollectionSpeed",Constants.AlgaeCollectionSpeed));
  Constants.AlgaeScoringSpeed = (SmartDashboard.getNumber("AlgaeScoringSpeed",Constants.AlgaeScoringSpeed));
  Constants.ElevatorUpSpeed = (SmartDashboard.getNumber("ElevatorUpSpeed",Constants.ElevatorUpSpeed));
  Constants.ElevatorDownSpeed = (SmartDashboard.getNumber("ElevatorDownSpeed",Constants.ElevatorUpSpeed));

  Constants.L1Height = (SmartDashboard.getNumber("L1Height",Constants.L1Height));
  Constants.L2Height = (SmartDashboard.getNumber("L2Height",Constants.L2Height));
  Constants.L3Height = (SmartDashboard.getNumber("L3Height",Constants.L3Height));
  Constants.L4Height = (SmartDashboard.getNumber("L4Height",Constants.L4Height));

  Constants.Pvar = (SmartDashboard.getNumber("ElevatorP", Constants.Pvar));
  Constants.Ivar = (SmartDashboard.getNumber("ElevatorI", Constants.Ivar));
  Constants.Dvar = (SmartDashboard.getNumber("ElevatorD", Constants.Dvar));



}
  );
}
            
            /* CORAL MECHANISM */
                  //score coral
                  public Command CoralScoringMethod() { 
                    return run(
                      () -> {          
                        CoralScoringMotor.set(Constants.CoralScoringSpeed);
                          }
                    );
                  }
        
        
        
                public Command EncoderCheck(){
                  return run(
        ()->{
    /* / //  EncoderDistance=ElevatorEncoder.getRate(); */
        SmartDashboard.putNumber("EncoderDistance", EncoderDistance);
}
        
          );
        }
    
          //get arm into scoring position
      //    public Boolean ArmScoringPosition() {
      //      if (CoralArmLimitSwitch.get()) //\
      //      {
      //          CoralArmMotor.set(0); //do not know until motor is mounted
      //      }
    //        {
   //             CoralArmMotor.set(Constants.CoralArmSpeed); 
   //         }
    //        return CoralArmLimitSwitch.get();
    //      }

          // competition seasoning
    
          //intake coral
          public Command CoralCollectionMethod() { 
            return run(
              () -> {
               CoralScoringMotor.set(Constants.CoralCollectionSpeed);
          });
        }
    
          //collect coral from station
          // public Boolean ArmCollectionPosition() {
          //   if (CoralArmLimitSwitch.get()) //
          //   {
          //       CoralArmMotor.set(0); //do not know until motor is mounted
          //   }
          //   else
          //   {
          //       CoralArmMotor.set(Constants.CoralCollectionArmSpeed); 
          //   }
          //   return CoralArmLimitSwitch.get();
          // }


//TESTING

// public Command LimitSwitchTest(){
//   return run(
//     () -> {
//   ElevatorBottomLimitSwitch.get();
//   SmartDashboard.putBoolean("Am I Working", ElevatorBottomLimitSwitch.get());
//     }); 
// }

                                                                                           
                     
                                          
     /* ALGAE MECHANISM */
          //collect algae
          public Command AlgaeCollectionMethod() {
          return run(
           () -> {
         // System.out.println("algae collection works");
            AlgaeMotorControl(Constants.AlgaeCollectionSpeed);
           }); 
          }
          
          //score algae (processor)
          public Command AlgaeScoringMethod()
          {
            return run (
              () -> {
            AlgaeMotorControl(Constants.AlgaeScoringSpeed);
       //     System.out.println("algae scoring works");
              });
          }
    
            //control both algae motors
          public void AlgaeMotorControl(double speed)
          {
            LeftAlgaeMotor.set(-speed);
            RightAlgaeMotor.set(speed);
          }

          public Command AlgaeStop(){
            return run(
              () -> {
                LeftAlgaeMotor.set(0);
                RightAlgaeMotor.set(0);
              }
            );
          }
    
    
    /* ELEVATOR */
          //UP
          //: Find elevator heights for all four reef levels and processor (5 total)
       //  public Command ElevatorUpMethod()
       //   {
      //      return run(
     //         () -> { 
    //            ElevatorMotor.set(Constants.ElevatorUpSpeed);
   //           System.out.println("elevator up works");
  //            }); find speed for elevator 
 //     }
      //DOWN
    // public Command ElevatorDownMethod()
   //    //elevator check method 
   // {
 //      return run(
//        () -> {
//ElevatorMotor.set(Constants.ElevatorDownSpeed);
   //     System.out.println("elevator down works");
 //       }
  //      );
         //opposite of elevator up (so elevator goes down)
//      }

      public Command ElevatorManual() //manually controls elevator by setting it to the joystick value
      {
        return run(
          () -> {
            ElevatorMotor.set((RobotContainer.mechController.getRightY())/2);
            if((RobotContainer.mechController.getRightY())/2 > -0.05 && (RobotContainer.mechController.getRightY())/2 < 0.05)
              {ElevatorMotor.set(0);}
            SmartDashboard.putNumber("ElevatorManualSpeed",(RobotContainer.mechController.getRightY())/2);  //Puts values to smart dashboard
            SmartDashboard.putNumber("ElevatorHeight",ElevatorEncoder.getPosition());
          }
        );
      }
      
      public Command CoralManual() //manually controls coral arm
      {
        return run(
          () -> {
            
         //   SmartDashboard.putNumber("VVristManualSpeed",(RobotContainer.mechController.getLeftY())/100);
            SmartDashboard.putNumber("VVristAbsoluteEncoder",(CoralArmEncoder.getPosition()));
        //    if (CoralArmMotor.getAbsoluteEncoder().getPosition()>Constants.CoralMaximum){
         //     CoralArmMotor.set(-0.1);
         //   }
         //   if (CoralArmMotor.getAbsoluteEncoder().getPosition()<Constants.CoralMinimum){
         //     CoralArmMotor.set(0.1);
         //   }
         //   else 
         //   {
            CoralArmMotor.set(-((RobotContainer.mechController.getLeftY())/20));
       //   }
        }
        );
      }

      public Command CoralStop(){
        return run(
          () -> {
              CoralScoringMotor.set(0);
          }
        );
      }
      //STOP
    //  public void ElevatorStopMethod() // may not need 
   //   {
   //   }

      //CHECKING IF ELEVATOR IS AT BOTTOM
      public Command ElevatorStopCommand()
      {
        return run(
        () -> {
          
        if (RobotContainer.ElevatorBottomLimitSwitch.get())
        
         ElevatorMotor.set(0);
          ElevatorMotor.getEncoder().setPosition(0);

        }
        );
      }

      // public boolean ElevatorPosition() // do not know if this works
      // {
      //   if (ElevatorBottomLimitSwitch.get())
      //   {
      //      ElevatorMotor.set(0);
      //       System.out.println("elevator stopped moving"); //
      //   } 
      //  // else 
      //  // {
      // //     ElevatorDownMethod();
      // //  }
      //   return ElevatorBottomLimitSwitch.get();

      //  }
      


public Command ElevatorPIDSetup(){
  return runOnce(
    () -> {
      System.out.println("wrist works ish");
      ElevatorMotor.getEncoder().setPosition(0);
      motorconfig.closedLoop
      .p(Constants.Pvar)
      .i(Constants.Ivar)
      .d(Constants.Dvar)
      .outputRange(-1,1);
      ElevatorMotor.configure(motorconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SmartDashboard.setDefaultNumber("targetposition",0);
    SmartDashboard.setDefaultNumber("TargetVelocity",0);
    SmartDashboard.setDefaultBoolean("Control Mode",false);
    SmartDashboard.setDefaultBoolean("Reset Encoder",false);
    System.out.println("AAAAAAAAAAAAAAAAAAAAA");
    }
);
}

public Command ElevatorPIDMovement(double setpoint, double VVristsetpoint){
  return run(
    () -> {
      
        //double targetPosition = Constants.TARGETPOSITION;
        ElevatorPID.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        SmartDashboard.putNumber("ElevatorHeight",ElevatorMotor.getEncoder().getPosition());

      if (ElevatorMotor.getEncoder().getPosition() < (setpoint+50) && ElevatorMotor.getEncoder().getPosition() > (setpoint-50)){
        VVristPID.setReference(VVristsetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        SmartDashboard.putNumber("VVristPosition",CoralArmMotor.getEncoder().getPosition());
      }
    }
  );
}
public Command elevatorStop()
{
  return run(
    () -> {
      ElevatorMotor.set(0);
    }
  );
}

public Command VVristPIDSetup(){
  return runOnce(
    () -> {
      System.out.println("wrist works ish");
      CoralArmMotor.getEncoder().setPosition(0);
      VVristMotorconfig.closedLoop
      .p(Constants.wPvar)
      .i(Constants.wIvar)
      .d(Constants.wDvar)
      .outputRange(-1,1);
      CoralArmMotor.configure(VVristMotorconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SmartDashboard.setDefaultNumber("targetposition",0);
    SmartDashboard.setDefaultNumber("TargetVelocity",0);
    SmartDashboard.setDefaultBoolean("Control Mode",false);
    SmartDashboard.setDefaultBoolean("Reset Encoder",false);
    System.out.println("AAAAAAAAAAAAAAAAAAAAA");
    }
);
}

// public Command VVristPIDMovement(double VVristsetpoint){
//   return run(
//     () -> {
      
//         //double targetPosition = Constants.TARGETPOSITION;
//         VVristPID.setReference(VVristsetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        
//         SmartDashboard.putNumber("VVristPosition",CoralArmMotor.getEncoder().getPosition());
        
//     }
//   );
// }


public Command StopMethod()
{
  return run(
    () -> {
      LeftAlgaeMotor.set(0);
      RightAlgaeMotor.set(0);
      CoralScoringMotor.set(0);
      CoralArmMotor.set(0);
      ElevatorMotor.set(0);
      System.out.println("stop 1 works");
    }
  );

}

// sets targetposition to the different heights
// public Command SetToL1()
// {
//   return run (
// () -> {
//   Constants.TARGETPOSITION = Constants.L1Height;
// }
//   );
// }

// public Command SetToL2()
// {
//   return run (
// () -> {
//   Constants.TARGETPOSITION = Constants.L2Height;
// }
//   );
// }

// public Command SetToL3()
// {
//   return run (
// () -> {
//   Constants.TARGETPOSITION = Constants.L3Height;
// }
//   );
// }

// public Command SetToL4()
// {
//   return run (
// () -> {
//   Constants.TARGETPOSITION = Constants.L4Height;
// }
//   );
// }

// public Command SetToCoralStation()
// {
//   return run (
// () -> {
//   Constants.TARGETPOSITION = Constants.CoralStationHeight;
// }
//   );
// }

/* DEEP CLIMB */

// press button
// check position
// grab on to cage
// lift robot
// activate limit switch and stop


// initialize components



/* Motors */
//LeftAlgaeMotor
//RightAlgaeMotor
//CoralScoringMotor
//CoralArmMotor 
//ElevatorMotor
//DeepClimbMotor
// servo motor for deep climb

/* Limit Switches */
//ElevatorBottomLimitSwitch
//DeepClimbArmLimitSwitch (prevent arm from hitting robot)
//DeepClimbCageLimitSwitch (sucessfully grab cage)
//CoralArmLimitSwitch

/* Encoders */
//CoralArmEncoder


// public Command SpeedMaths(){
//   return run(
//     () -> {
// ElevatorMotor.getAbsoluteEncoder();      

//     }); 
// }

}







//why does thiis comment exist