// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DeepClimbMechanismSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ReefMechanismSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ReefMechanismSubsystem reefSubsystem = new ReefMechanismSubsystem();
  private final DeepClimbMechanismSubsystem deepClimbSubsystem = new DeepClimbMechanismSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandXboxController mechController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
  //  new Trigger(reefSubsystem).onTrue(new ExampleCommand(reefSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    
    if (mechController.getRightY()<0.5 && mechController.getLeftY()<0.5){

      //mechController.x().whileTrue(reefSubsystem.SetToL2());
      mechController.x().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.L2Height));
      mechController.x().whileTrue(reefSubsystem.VVristPIDMovement(Constants.L2Angle));

      //mechController.y().whileTrue(reefSubsystem.SetToL4());
      mechController.y().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.L4Height));
      mechController.y().whileTrue(reefSubsystem.VVristPIDMovement(Constants.L4Angle));

      //mechController.a().whileTrue(reefSubsystem.SetToL1());
      mechController.a().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.L1Height));
      mechController.a().whileTrue(reefSubsystem.VVristPIDMovement(Constants.L1Angle));

      //mechController.b().whileTrue(reefSubsystem.SetToL3());
      mechController.b().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.L3Height));
      mechController.b().whileTrue(reefSubsystem.VVristPIDMovement(Constants.L3Angle));

      mechController.back().whileTrue(reefSubsystem.ElevatorPIDMovement(Constants.CoralStationHeight));
      mechController.back().whileTrue(reefSubsystem.VVristPIDMovement(Constants.CoralStationAngle));
      //  mechController.back().whileTrue(reefSubsystem.ElevatorPIDMovement());
    }
    else{
      reefSubsystem.ElevatorManual();
      reefSubsystem.CoralManual();
    }

                                    
    

    mechController.rightBumper().whileTrue(reefSubsystem.AlgaeCollectionMethod());

    mechController.rightTrigger().whileTrue(reefSubsystem.AlgaeScoringMethod()); 

    mechController.povLeft().whileTrue(deepClimbSubsystem.DeepClimbGrab());

    mechController.povUp().whileTrue(deepClimbSubsystem.DeepClimbLift());

    mechController.leftBumper().whileTrue(reefSubsystem.CoralCollectionMethod());

    mechController.leftTrigger().whileTrue(reefSubsystem.CoralScoringMethod());

    mechController.start().whileTrue(reefSubsystem.StopMethod());
    mechController.start().whileTrue(deepClimbSubsystem.DeepClimbStopMethod());
    


    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
