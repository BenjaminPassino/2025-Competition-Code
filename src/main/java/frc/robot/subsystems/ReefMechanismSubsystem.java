package frc.robot.subsystems;


import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

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
    private final static SparkMax ElevatorMotor = new SparkMax(Constants.ElevatorMotorPort,MotorType.kBrushless);
    
        /* Limit Switch */
        private final DigitalInput ElevatorBottomLimitSwitch = new DigitalInput(Constants.ElevatorBottomLimitSwitchPort);
       
// that feeling when you and your friends take a boat to Istanbul and eat bacon on top of a mountain bc you were bored in history class
 
        private final DigitalInput CoralArmLimitSwitch = new DigitalInput(Constants.CoralArmLimitSwitchPort);
    
        /* Encoders */
        private final Encoder CoralArmEncoder = new Encoder(5, 6);
        private final Encoder ElevatorEncoder = new Encoder(7,8);
        private double EncoderDistance =1;
        private SparkMaxConfig motorconfig;
        private SparkClosedLoopController ElevatorPID = ElevatorMotor.getClosedLoopController();
        
    

        
            
               // public Command mechanismInitializeCommand() {
                    // Inline construction of command goes here.
                    // Subsystem::RunOnce implicitly requires `this` subsystem.
                    
              //    }
            
            public Command Setup(){
              return run(
                () -> {
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
              
          

  }
);
          }

public Command Updater(){ //updates smart dashboard values
  return run(
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

}
  );
}
            
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
            System.out.println("algae scoring works");
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

      public Command ElevatorManual()
      {
        return run(
          () -> {
            ElevatorMotor.set((RobotContainer.mechController.getRightY())/2);
          }
        );
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
      


public Command ElevatorPIDSetup(){
  return run(
    () -> {
      motorconfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-1,1);
      ElevatorMotor.configure(motorconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SmartDashboard.setDefaultNumber("targetposition",0);
    SmartDashboard.setDefaultNumber("TargetVelocity",0);
    SmartDashboard.setDefaultBoolean("Control Mode",false);
    SmartDashboard.setDefaultBoolean("Reset Encoder",false);
    }
);
}

public Command ElevatorPIDMovement(double setpoint){
  return run(
    () -> {
      
        //double targetPosition = Constants.TARGETPOSITION;
        ElevatorPID.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);

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
public Command SetToL1()
{
  return run (
() -> {
  Constants.TARGETPOSITION = Constants.L1Height;
}
  );
}

public Command SetToL2()
{
  return run (
() -> {
  Constants.TARGETPOSITION = Constants.L2Height;
}
  );
}

public Command SetToL3()
{
  return run (
() -> {
  Constants.TARGETPOSITION = Constants.L3Height;
}
  );
}

public Command SetToL4()
{
  return run (
() -> {
  Constants.TARGETPOSITION = Constants.L4Height;
}
  );
}

public Command SetToCoralStation()
{
  return run (
() -> {
  Constants.TARGETPOSITION = Constants.CoralStationHeight;
}
  );
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
