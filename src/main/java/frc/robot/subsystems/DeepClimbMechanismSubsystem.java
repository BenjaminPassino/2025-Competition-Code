package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class DeepClimbMechanismSubsystem extends SubsystemBase{

       // Deep Climb
       private final static PWMSparkMax DeepClimbMotor = new PWMSparkMax(Constants.DeepClimbMotorPort);
       private final static Servo DeepClimbServo = new Servo(Constants.DeepClimbServoPort);

       //limit switch
       private final DigitalInput DeepClimbArmLimitSwitch = new DigitalInput(Constants.DeepClimbArmLimitSwitchPort);
       private final DigitalInput DeepClimbCageLimitSwitch = new DigitalInput(Constants.DeepClimbCageLimitSwitchPort);

       public Command DeepClimbGrab() {
              return run(
              () -> {
                     if (DeepClimbCageLimitSwitch.get())
                     {
                    System.out.println("deep climb grabbing works");
                     DeepClimbServo.set(.2);

                     }
              }
              );
       }

       public boolean DeepClimbGrabPosition(){
       if (DeepClimbCageLimitSwitch.get())
            {
                DeepClimbServo.set(0); //do not know until motor is mounted
            }
            else
            {
                DeepClimbServo.set(.05); 
            }
            return DeepClimbCageLimitSwitch.get();
          }


//grab cage
//stop once cage is grabbed
//pull robot up
//trigger limit switch and stop


}