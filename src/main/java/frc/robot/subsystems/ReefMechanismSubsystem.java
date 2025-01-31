package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReefMechanismSubsystem extends SubsystemBase{
    /* Motors */
    //algae motors
    private final static PWMSparkMax LeftAlgaeMotor = new PWMSparkMax(Constants.LeftAlgaeMotorPort);
    private final static PWMSparkMax RightAlgaeMotor = new PWMSparkMax(Constants.RightAlgaeMotorPort);
    //Coral motors
    private final static PWMSparkMax CoralScoringMotor = new PWMSparkMax(Constants.CoralScoringMotorPort);
    private final static PWMSparkMax CoralArmMotor = new PWMSparkMax(Constants.CoralArmMotorPort);
    //Elevator motor
    private final static PWMSparkMax ElevatorMotor = new PWMSparkMax(Constants.ElevatorMotorPort);
    
        /* Limit Switch */
        private final DigitalInput ElevatorBottomLimitSwitch = new DigitalInput(Constants.ElevatorBottomLimitSwitchPort);
       
// that feeling when you and your friends take a boat to Istanbul and eat bacon on top of a mountain bc you were bored in history class

        private final DigitalInput CoralArmLimitSwitch = new DigitalInput(Constants.CoralArmLimitSwitchPort);
    
        /* Encoders */
        private final Encoder CoralArmEncoder = new Encoder(5, 6);
        private final Encoder ElevatorEncoder = new Encoder(7,8);
        private double EncoderDistance =1;
            
               // public Command mechanismInitializeCommand() {
                    // Inline construction of command goes here.
                    // Subsystem::RunOnce implicitly requires `this` subsystem.
                    
              //    }
            
            
            /* CORAL MECHANISM */
                  //score coral
                  public Command CoralScoringMethod() { 
                    return run(
                      () -> {
                        System.out.println("coral scoring works");
                    if (ArmScoringPosition())
                    {
                        CoralScoringMotor.set(Constants.CoralScoringSpeed);
                    }});
                  }
        
        
        
                public Command EncoderCheck(){
                  return run(
        ()->{
        EncoderDistance=ElevatorEncoder.getRate();
        SmartDashboard.putNumber("EncoderDistance", EncoderDistance);
}
        
          );
        }
    
          //get arm into scoring position
          public Boolean ArmScoringPosition() {
            if (CoralArmLimitSwitch.get())
            {
                CoralArmMotor.set(0); //do not know until motor is mounted
            }
            {
                CoralArmMotor.set(Constants.CoralArmSpeed); 
            }
            return CoralArmLimitSwitch.get();
          }

          // competition seasoning
    
          //intake coral
          public Command CoralCollectionMethod() { 
            return run(
              () -> {
                System.out.println("coral collection works");
            if (ArmCollectionPosition())
            {
                CoralScoringMotor.set(Constants.CoralScoringSpeed);
            } });
        }
    
          //collect coral from station
          public Boolean ArmCollectionPosition() {
            if (CoralArmLimitSwitch.get())
            {
                CoralArmMotor.set(0); //do not know until motor is mounted
            }
            else
            {
                CoralArmMotor.set(Constants.CoralCollectionArmSpeed); 
            }
            return CoralArmLimitSwitch.get();
          }


//TESTING

public Command LimitSwitchTest(){
  return run(
    () -> {
  ElevatorBottomLimitSwitch.get();
  SmartDashboard.putBoolean("Am I Working", ElevatorBottomLimitSwitch.get());
    }); 
}

                                                                                           
                     
                                          
     /* ALGAE MECHANISM */
          //collect algae
          public Command AlgaeCollectionMethod() {
          return run(
           () -> {
          System.out.println("algae collection works");
            AlgaeMotorControl(Constants.AlgaeCollectionSpeed);
           }); 
          }
          
          //score algae (processor)
          public Command AlgaeScoringMethod()
          {
            return run (
              () -> {
            AlgaeMotorControl(Constants.AlgaeScoringSpeed);
              });
          }
    
            //control both algae motors
          public void AlgaeMotorControl(double speed)
          {
            LeftAlgaeMotor.set(-speed); //TODO check motor direction
            RightAlgaeMotor.set(speed);
          }
    
    
    /* ELEVATOR */
          //UP
          //TODO: Find elevator heights for all four reef levels and processor (5 total)
          public Command ElevatorUpMethod()
          {
            return run(
              () -> { 
                ElevatorMotor.set(Constants.ElevatorUpSpeed);
              System.out.println("elevator up works");
              });//TODO find speed for elevator 
      }
      //DOWN
     public Command ElevatorDownMethod()
       //elevator check method 
    {
       return run(
        () -> {
          ElevatorMotor.set(Constants.ElevatorDownSpeed);
        System.out.println("elevator down works");
        }
        );
         //opposite of elevator up (so elevator goes down)
      }

      //STOP
    //  public void ElevatorStopMethod() // may not need 
   //   {
   //   }
// TODO FINISH CODING IN LIMIT SWITCH!!!
      //CHECKING IF ELEVATOR IS AT BOTTOM
      public boolean ElevatorPosition() // do not know if this works
      {
        if (ElevatorBottomLimitSwitch.get())
        {
           ElevatorMotor.set(0);
            System.out.println("elevator stopped working");
        } 
       // else 
       // {
      //     ElevatorDownMethod();
      //  }
        return ElevatorBottomLimitSwitch.get();
       }
      

/* DEEP CLIMB */

// press button
// check position
// grab on to cage
// lift robot
// activate limit switch and stop
}

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
