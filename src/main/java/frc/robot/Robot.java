// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//HEllo
//hi
package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

@Override
public void robotInit() {

  //FollowPathCommand.warmupCommand().schedule();
}


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
 


  }

  @Override
  public void disabledInit() {}

  /* 
  public void PowerDistribution(){

    PowerDistribution examplePD = new PowerDistribution();

     double voltage = examplePD.getVoltage();
     double TempCelsius = examplePD.getTemperature();

     double totalCurrent = examplePD.getTotalCurrent();

     double totalPower = examplePD.getTotalPower();

     double totalEnergy = examplePD.getTotalEnergy();

     double current7 = examplePD.getCurrent(7);
     
     SmartDashboard.putNumber("Temp",TempCelsius);
     SmartDashboard.putNumber("Voltage",voltage);
     SmartDashboard.putNumber("Current",totalCurrent);
     SmartDashboard.putNumber("Power",totalPower);
     SmartDashboard.putNumber("Energy",totalEnergy);
     SmartDashboard.putNumber("Current7",current7);
  
  examplePD.setSwitchableChannel(true);
*/
   // }
  
  
  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
   // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

  //  if (m_autonomousCommand != null) {
   //   m_autonomousCommand.schedule();
  //  }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

   // PowerDistribution();
  }



  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
