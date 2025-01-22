package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
        // Deep Climb
        private final static PWMSparkMax DeepClimbMotor = new PWMSparkMax(Constants.DeepClimbMotorPort);
        //deep climb servo motor
    
        /* Limit Switch */
        private final DigitalInput ElevatorBottomLimitSwitch = new DigitalInput(Constants.ElevatorBottomLimitSwitchPort);
        private final DigitalInput DeepClimbArmLimitSwitch = new DigitalInput(Constants.DeepClimbArmLimitSwitchPort);
        private final DigitalInput DeepClimbCageLimitSwitch = new DigitalInput(Constants.DeepClimbCageLimitSwitchPort);
        private final DigitalInput CoralArmLimitSwitch = new DigitalInput(Constants.CoralArmLimitSwitchPort);
    
        /* Encoders */
        private final Encoder CoralArmEncoder = new Encoder(5, 6);
    
       // public Command mechanismInitializeCommand() {
            // Inline construction of command goes here.
            // Subsystem::RunOnce implicitly requires `this` subsystem.
            
      //    }
    
          /* CONTROLS */
    
      
    
    // call buttons here
    // learn by 1/21
    
    /* CORAL MECHANISM */
          //score coral
          public Command CoralScoringMethod() { 
            //TODO figure out how to call
            return run(
              () -> {
                System.out.println("coral scoring works");
            if (ArmScoringPosition())
            {
                CoralScoringMotor.set(.4);
            }});
          }
    
          //get arm into scoring position
          public Boolean ArmScoringPosition() {
            if (CoralArmLimitSwitch.get())
            {
                CoralArmMotor.set(0); //do not know until motor is mounted
            }
            else
            {
                CoralArmMotor.set(.2); // TODO do not know
            }
            return CoralArmLimitSwitch.get();
          }
    
          //intake coral
          public Command CoralCollectionMethod() { 
            return run(
              () -> {
                System.out.println("coral collection works");
            if (ArmCollectionPosition())
            {
                CoralScoringMotor.set(.4);
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
                CoralArmMotor.set(.2); // TODO do not know
            }
            return CoralArmLimitSwitch.get();
          }
                                                                                           
                     
                                          
     /* ALGAE MECHANISM */
          //collect algae
          public Command AlgaeCollectionMethod() {
          return run(
           () -> {
          System.out.println("algae collection works");
            AlgaeMotorControl(.2);
           }); 
          }
          
          //score algae (processor)
          public Command AlgaeScoringMethod()
          {
            return run (
              () -> {
            AlgaeMotorControl(-0.2);
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
                ElevatorMotor.set(.2);
              System.out.println("elevator up works");
              });//TODO find speed for elevator 
      }
      //DOWN
     public Command ElevatorDownMethod()
       //elevator check method 
    {
       return run(
        () -> {
          ElevatorMotor.set(-.2);
        System.out.println("elevator down works");
        }
        );
         //opposite of elevator up (so elevator goes down)
      }

      //STOP
      public void ElevatorStopMethod() // may not need 
      {

      }

      //CHECKING IF ELEVATOR IS AT BOTTOM
   //   public boolean ElevatorPosition() // do not know if this works
   //   {
    //    if (ElevatorBottomLimitSwitch.get())
   //     {
//            ElevatorMotor.set(0);
  //          System.out.println("elevator stop works");
  //      }
    //    else 
   //     {
 //           ElevatorDownMethod();
   //     }
     //   return ElevatorBottomLimitSwitch.get();
      // }
      

/* DEEP CLIMB */

// press buttom
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

// limit switch or encoder for coral
// four settings for elevator height
//  
