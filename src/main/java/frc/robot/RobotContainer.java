// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveMotors;
import frc.robot.subsystems.TestMotorsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  private TestMotorsSubsystem testMotor;
      
  public RobotContainer() {
    initSubsystems();    
    // Configure the trigger bindings
    configureBindings();
  }

  private void initSubsystems() {
      testMotor = new TestMotorsSubsystem(10);
  }

  private void configureBindings() {
    int preset = 1;
    switch (preset) {
            default:
                    controllerPresetMain();
                    break;
    }
  }

  public Command getAutonomousCommand() {
        return null;
  }

  public void controllerPresetMain() { //subject to change (while/on true)
        double speed = 0.35;
        m_driverController.y().whileTrue(new DriveMotors(testMotor, speed));
      //   m_driverController.y().whileTrue(new DriveMotors(frontRightTurn, speed));
      //   m_driverController.x().whileTrue(new DriveMotors(frontRightTurn, -speed));
        
      //   m_driverController.b().whileTrue(new DriveMotors(rearRightDrive, speed));
      //   m_driverController.a().whileTrue(new DriveMotors(rearRightDrive, -speed));
  }

}