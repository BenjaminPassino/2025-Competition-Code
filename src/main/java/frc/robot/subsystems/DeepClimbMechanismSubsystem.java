package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.ClosedLoopSlot;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// hi



public class DeepClimbMechanismSubsystem extends SubsystemBase{

       // Deep Climb
       private final static SparkMax DeepClimbMotor = new SparkMax(Constants.DeepClimbMotorPort,MotorType.kBrushless);
       private final static Servo DeepClimbServo = new Servo(Constants.DeepClimbServoPort);

       //limit switch
    //   private final DigitalInput DeepClimbArmLimitSwitch = new DigitalInput(Constants.DeepClimbArmLimitSwitchPort);
       private final DigitalInput DeepClimbCageLimitSwitch = new DigitalInput(Constants.DeepClimbCageLimitSwitchPort);

       
       private SparkMaxConfig climbmotorconfig;
        private SparkClosedLoopController ClimberPID = DeepClimbMotor.getClosedLoopController();

       public Command DeepClimb(){ // full mechanism
              return run(
                     () -> {
                            if (DeepClimbCageLimitSwitch.get()) //checks for limit switch
                     DeepClimbLift(); // lifts if limit switch has been triggered
                     else{
                   DeepClimbGrab();   // grabs cage if it has not been grabbed
                     } 
                     }
              );
       }

       public Command DeepClimbRelease(){
              return run(
              () -> {
                     DeepClimbMotor.set(Constants.ReleasePortSpeed);
              }
              );
       }

       // may not work
       public Command DeepClimbGrab() { // grabs onto cage with ratchet
              return run(
              () -> {
                    System.out.println("deep climb grabbing works");
                     DeepClimbServo.set(.2);
                     RobotContainer.mechController.setRumble(RumbleType.kBothRumble, 1);
                     }  
              );
       }
       public Command DeepClimbNotGrab() { // grabs onto cage with ratchet
              return run(
              () -> {
                    System.out.println("deep climb grabbing works");
                     DeepClimbServo.set(.4);
                     RobotContainer.mechController.setRumble(RumbleType.kBothRumble, 0);       
              }  
              );
       }


       public boolean DeepClimbGrabPosition(){
       if (DeepClimbCageLimitSwitch.get())
            {
                DeepClimbServo.set(0); } // checks if the ratchet is in place
   //         else
     //       {
  //              DeepClimbServo.set(.05); 
//            }
            return DeepClimbCageLimitSwitch.get();    
          }

    //      public boolean DeepClimbLiftPosition(){
      //        if (DeepClimbArmLimitSwitch.get())
              {
                     DeepClimbMotor.set(0);
                     System.out.println("deep climb lift stop works");
              }
     //         return DeepClimbArmLimitSwitch.get();
     //     }

       
          public Command DeepClimbLift(){
              return run(
                  () -> {
                    // if (DeepClimbCageLimitSwitch.get()); //checks for coral limit switch
                     System.out.println("deep climb lift works");
                     DeepClimbMotor.set(.8); // lifts robot when ratchet is in place
                  }   
              );
          }
          // rumble when ratchet grabs cage

          public Command DeepClimbStopMethod(){
              return run(
              () -> {
                     DeepClimbMotor.set(0);
                     System.out.println("stop 2 works");

              }

              );
          }




          public Command ClimberPIDSetup(){
  return run(
    () -> {
      climbmotorconfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.cPvar)
      .i(Constants.cIvar)
      .d(Constants.cDvar)
      .outputRange(-1,1);
      DeepClimbMotor.configure(climbmotorconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SmartDashboard.setDefaultNumber("targetposition",0);
    SmartDashboard.setDefaultNumber("TargetVelocity",0);
    SmartDashboard.setDefaultBoolean("Control Mode",false);
    SmartDashboard.setDefaultBoolean("Reset Encoder",false);
    }
);
}
//grab cage
//stop once cage is grabbed
//pull robot up
//trigger limit switch and stop


}