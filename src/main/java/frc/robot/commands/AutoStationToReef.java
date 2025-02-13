package frc.robot.commands;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ReefMechanismSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;



public class AutoStationToReef extends Command{


 @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ReefMechanismSubsystem reefSubsystem;

public Timer AutonomousTimer = new Timer();
public Timer CoralTimer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoStationToReef(ReefMechanismSubsystem subsystem) {
    reefSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CoralTimer.start();
  //  if (AutonomousTimer.get() > Constants.EndRelease)
  ///  {reefSubsystem.elevatorStop();}
   // else
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    reefSubsystem.ElevatorPIDMovement(Constants.L4Height);
    if (CoralTimer.get() > Constants.StationToReef)
        reefSubsystem.CoralScoringMethod();
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CoralTimer.stop();
    CoralTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}






