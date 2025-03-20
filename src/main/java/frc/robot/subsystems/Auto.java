package frc.robot.subsystems;

// import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.RobotContainer;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkClosedLoopController;

// import javax.lang.model.util.ElementScanner14;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

// import frc.robot.Constants;

public class Auto extends SubsystemBase{

//evan was here hehe :)

// public Timer AutonomousTimer = new Timer();
// public Timer CoralTimer = new Timer();
// public boolean ReadThrough = false;
// private final ReefMechanismSubsystem reefSubsystem = new ReefMechanismSubsystem();



// public Command TimerStart(){
//     return runOnce(
//     () -> {
//         AutonomousTimer.start();
//         System.out.println("timer started");
//         reefSubsystem.ElevatorPIDSetup();
//           }
//     );   
// }

// public Command AlgaeRelease(){
//     return run(
//         () -> {
            
//             if (AutonomousTimer.get() > Constants.EndRelease)
//             {reefSubsystem.elevatorStop();}

//             else {
//                 reefSubsystem.ElevatorUpMethod();
//             }
//         }
  //  );
//}

// public Command AutoL4(){
//     return run(
//         () -> {
//             CoralTimer.start();
//             reefSubsystem.ElevatorPIDMovement(Constants.L4Height);
//             if (CoralTimer.get() > 1.5){
//                 reefSubsystem.CoralScoringMethod();
//                 CoralTimer.stop();
//                 CoralTimer.reset();
//                 return true;
//             }
//             return false;
//         }
//     );
// }













}
