package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class DeepClimbMechanismSubsystem extends SubsystemBase{

       // Deep Climb
       private final static PWMSparkMax DeepClimbMotor = new PWMSparkMax(Constants.DeepClimbMotorPort);
       
       //deep climb servo motor

       //limit switch
       private final DigitalInput DeepClimbArmLimitSwitch = new DigitalInput(Constants.DeepClimbArmLimitSwitchPort);
       private final DigitalInput DeepClimbCageLimitSwitch = new DigitalInput(Constants.DeepClimbCageLimitSwitchPort);


//grab cage
//stop once cage is grabbed
//pull robot up
//trigger limit switch and stop


}