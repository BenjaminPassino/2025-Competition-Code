// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.events.EventTrigger;
//import com.pathplanner.lib.events.PointTowardsZoneTrigger;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DeepClimbMechanismSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ReefMechanismSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCoralCollection;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutoCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.MechCamera;
//import com.pathplanner.lib.pathfinding;




public class RobotContainer {
  private final ReefMechanismSubsystem reefSubsystem = new ReefMechanismSubsystem();
  private final DeepClimbMechanismSubsystem deepClimbSubsystem = new DeepClimbMechanismSubsystem();
  public static DigitalInput ElevatorBottomLimitSwitch = new DigitalInput(Constants.ElevatorBottomLimitSwitchPort);

  //NamedCommands.registerCommand("CoralCollection", reefSubsystem.AlgaeCollectionMethod() );
 
   // new PointTowardsZoneTrigger("Speaker").whileTrue(Commands.print("aiming at speaker"));

    //new EventTrigger("run intake").whileTrue(Commands.print("running intake"));
   
   // new EventTrigger("ElevatorStop").whileTrue(reefSubsystem.ElevatorStopCommand());

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

   // private final SendableChooser<Command> autoChooser;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final MechCamera mechCamera = new MechCamera();
 public final static CommandXboxController mechController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
    configureButtonBindings();
}

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
      
        mechCamera.setDefaultCommand(mechCamera.Camera());

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-joystick.getLeftY() * MaxSpeed *.5)) // Drive forward with negative Y (forward)
                    .withVelocityY((-joystick.getLeftX() * MaxSpeed* .5)) // Drive left with negative X (left)
                    .withRotationalRate((-joystick.getRightX() * MaxAngularRate *.5 )) // Drive counterclockwise with negative X (left)
                    )
        );
 
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    

     
    
    }

   private void configureButtonBindings(){

    //if (mechController.getRightY()<0.2){

      //mechController.x().whileTrue(reefSubsystem.SetToL2());
     mechController.x().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.L2Height)).whileTrue(reefSubsystem.VVristPIDMovement(Constants.L2Angle));

      //mechController.y().whileTrue(reefSubsystem.SetToL4());
      mechController.y().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.L4Height)).whileTrue(reefSubsystem.VVristPIDMovement(Constants.L4Angle));

      //mechController.a().whileTrue(reefSubsystem.SetToL1());
      mechController.a().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.L1Height)).whileTrue(reefSubsystem.VVristPIDMovement(Constants.L1Angle));

      //mechController.b().whileTrue(reefSubsystem.SetToL3());
      mechController.b().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.L3Height)).whileTrue(reefSubsystem.VVristPIDMovement(Constants.L3Angle));

   //   mechController.back().and(ElevatorLimitSwitchTrigger).whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.CoralStationHeight));
      mechController.back().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.CoralStationHeight)).whileTrue(reefSubsystem.VVristPIDMovement(Constants.CoralStationAngle));
    //}
   // else{
      mechController.rightStick().whileTrue(reefSubsystem.ElevatorManual());
  //  }

  //  if (mechController.getLeftY()<0.2)
//{
  
//}
//  else{
  mechController.leftStick().whileTrue(reefSubsystem.CoralManual());
// }
    mechController.rightBumper().whileTrue(reefSubsystem.AlgaeCollectionMethod()).onFalse(reefSubsystem.AlgaeStop());

    mechController.rightTrigger().whileTrue(reefSubsystem.AlgaeScoringMethod()).onFalse(reefSubsystem.AlgaeStop());

    mechController.povLeft().whileTrue(deepClimbSubsystem.DeepClimbNotGrab());
  

    mechController.povUp().whileTrue(deepClimbSubsystem.DeepClimbLift());
    mechController.povCenter().whileTrue(deepClimbSubsystem.DeepClimbStopMethod());
    
  //      .or(mechController.povDown()).whileTrue(deepClimbSubsystem.DeepClimbRelease())
  //      .whileFalse(deepClimbSubsystem.DeepClimbStopMethod());//rotate clockwise

    mechController.povRight().whileTrue(deepClimbSubsystem.DeepClimbGrab());//ratchet

    mechController.povDown().whileTrue(deepClimbSubsystem.DeepClimbRelease());

    mechController.leftBumper().whileTrue(reefSubsystem.CoralCollectionMethod()).onFalse(reefSubsystem.CoralStop());

    mechController.leftTrigger().whileTrue(reefSubsystem.CoralScoringMethod()).onFalse(reefSubsystem.CoralStop());

    mechController.start().whileTrue(reefSubsystem.StopMethod()).whileTrue(deepClimbSubsystem.DeepClimbStopMethod());
  
  //joystick.y();
  //joystick.y().onTrue(reefSubsystem.ElevatorPIDSetup());
  joystick.y().onTrue(reefSubsystem.Setup()).onTrue(reefSubsystem.VVristPIDSetup()).onTrue(reefSubsystem.ElevatorPIDSetup());
  
   }


/* 
    public RobotContainer() {
    
 
        new PointTowardsZoneTrigger("Speaker").whileTrue(Commands.print("aiming at speaker"));

        new EventTrigger("run intake").whileTrue(Commands.print("running intake"));
    
        
    
            // Build an auto chooser. This will use Commands.none() as the default option.
        //autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    //SmartDashboard.putData("Auto Chooser", autoChooser);

  
   



  }}

  //public Command getAutonomousCommand() {

    try{
        // Load the path you want to follow using its name in the GUI
      //  PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
}


}

*/

}