// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController controller =
      new CommandXboxController(Shared.OperatorConstants.DRIVER_CONTROLLER_PORT);
  
  private KitbotIntakeSubsystem intakeSub;
  private KitbotShootingSubsystem shootSub;

  public RobotContainer() {
    intakeSub = new KitbotIntakeSubsystem();
    shootSub = new KitbotShootingSubsystem();
     
    configureBindings();
  }

  private void configureBindings() {
      intakeSub.setDefaultCommand(new RunCommand(
        () -> intakeSub.stopIntake(), intakeSub));
      shootSub.setDefaultCommand(new RunCommand(
        () -> shootSub.stopMotors(), shootSub));
      
      controller.b().whileTrue(new RunCommand(
        () -> intakeSub.intake(), intakeSub));
      controller.a().whileTrue(new RunCommand(
        () -> intakeSub.setFeederRoller(1), intakeSub));
      controller.y().whileTrue(new RunCommand(
        () -> shootSub.shooting(), intakeSub));
  }
}
