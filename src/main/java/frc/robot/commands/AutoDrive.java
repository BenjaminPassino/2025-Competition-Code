package frc.robot.commands;

import frc.robot.subsystems.ReefMechanismSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class AutoDrive extends Command {
 @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

public Timer AutonomousTimer = new Timer();
public Timer CoralTimer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



