package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefMechanismSubsystem extends SubsystemBase{
    //algae motors
    private final PWMSparkMax LeftAlgaeMotor = new PWMSparkMax(1);
    private final PWMSparkMax RightAlgaeMotor = new PWMSparkMax(2);
    //Coral motors
    private final PWMSparkMax CoralScoringMotor = new PWMSparkMax(3);
    private final PWMSparkMax CoralArmMotor = new PWMSparkMax(4);
    //Elevator motor
    private final PWMSparkMax ElevatorMotor = new PWMSparkMax(5);
    // Deep Climb
    private final PWMSparkMax DeepClimbMotor = new PWMSparkMax(6);
    // limit switch
    private final DigitalInput ElevatorBottomLimitSwitch = new DigitalInput(0);
    private final DigitalInput DeepClimbArmLimitSwitch = new DigitalInput(1);
    private final DigitalInput DeepClimbCageLimitSwitch = new DigitalInput(2);
    //encoder
    private final Encoder CoralArmEncoder = new Encoder(3, 4);

    public Command mechanismInitializeCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
              /* one-time action goes here */
              
            });
      }

}

// initialize components



//LeftAlgaeMotor
//RightAlgaeMotor
//CoralScoringMotor
//CoralArmMotor 
//CoralArmEncoder
//ElevatorMotor
//ElevatorBottomLimitSwitch
//DeepClimbArmLimitSwitch
//DeepClimbCageLimitSwitch
//DeepClimbMotor

// limit switch or encoder for coral
// four settings for elevator height
//  
