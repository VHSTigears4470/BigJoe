// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.*;

public class RobotContainer {
  private DriveSubsystem driveSub;
  private VisionSubsystem visionSub;
  private ShooterSubsystem shooterSub;
  private ClimbSubsystem climbSub;
  private IntakeSubsystem intakeSub;

  private final CommandXboxController driverController = new CommandXboxController(OI.Constants.DRIVE_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OI.Constants.OPERATOR_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    initSubystems();

    if (Operating.Constants.USING_DRIVE) {
      autoChooser = AutoBuilder.buildAutoChooser();
      // Add paths here
      autoChooser.addOption("1", new PathPlannerAuto("1"));
      autoChooser.addOption("ShootCenter", new PathPlannerAuto("ShootCenter"));
      autoChooser.addOption("ShootLeft", new PathPlannerAuto("ShootLeft"));
      autoChooser.addOption("ShootRight", new PathPlannerAuto("ShootRight"));
      autoChooser.addOption("Collect Frame", new PathPlannerAuto("Collect Frame"));
      autoChooser.addOption("ShootCenter Manual", new PathPlannerAuto("ShootCenter Manual"));
      SmartDashboard.putData("Auto Mode", autoChooser);
    } else {
      autoChooser = null;
    }

    configureBindings();

    FollowPathCommand.warmupCommand().schedule();
  }

  public void initSubystems() {
    if (Operating.Constants.USING_VISION) {
      visionSub = new VisionSubsystem();
    }

    if (Operating.Constants.USING_DRIVE) {
      driveSub = new DriveSubsystem(Optional.ofNullable(visionSub));
      driveSub.setDefaultCommand(new RunCommand(
          () -> {
            double y = OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil
                .applyDeadband(driverController.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND);
            double x = OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil
                .applyDeadband(driverController.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND);
            double rot = OI.Constants.DRIVER_AXIS_ROT_INVERTED * MathUtil
                .applyDeadband(driverController.getRawAxis(OI.Constants.DRIVER_AXIS_ROT), OI.Constants.DRIVE_DEADBAND);

            // Add logging for buttons

            // Record operator inputs with the project logger
            Logger.recordOutput("Operator/Drive/Y", y);
            Logger.recordOutput("Operator/Drive/X", x);
            Logger.recordOutput("Operator/Drive/Rot", rot);
            Logger.recordOutput("Operator/Drive/LeftTrigger", driverController.leftTrigger().getAsBoolean());
            Logger.recordOutput("Operator/Drive/RightTrigger", driverController.rightTrigger().getAsBoolean());

            driveSub.drive(y, x, rot, true, "Default / Field Oriented");
          },
          driveSub));

      NamedCommands.registerCommand("ToggleDriveAligned", new DriveAlignedToggle(driveSub));

      PathConstraints constraints = new PathConstraints(
          2.0, 2.0,
          Units.degreesToRadians(240),
          Units.degreesToRadians(240));
      PathPlannerPath path;
      
      try {
        path = PathPlannerPath.fromPathFile("ToFrame");
      } catch (Exception e) {
        path = null;
      }
      NamedCommands.registerCommand("moveToFrame", AutoBuilder.pathfindThenFollowPath(path, constraints));

      try {
        path = PathPlannerPath.fromPathFile("ShootRight");
      } catch (Exception e) {
        path = null;
      }
      NamedCommands.registerCommand("ShootRight", AutoBuilder.pathfindThenFollowPath(path, constraints));

      try {
        path = PathPlannerPath.fromPathFile("ShootCenter");
      } catch (Exception e) {
        path = null;
      }
      NamedCommands.registerCommand("ShootCenter", AutoBuilder.pathfindThenFollowPath(path, constraints));

      try {
        path = PathPlannerPath.fromPathFile("ShootLeft");
      } catch (Exception e) {
        path = null;
      }
      NamedCommands.registerCommand("ShootLeft", AutoBuilder.pathfindThenFollowPath(path, constraints));

      try {
        path = PathPlannerPath.fromPathFile("ClimbLeft");
      } catch (Exception e) {
        path = null;
      }
      NamedCommands.registerCommand("ClimbLeft", AutoBuilder.pathfindThenFollowPath(path, constraints));

      try {
        path = PathPlannerPath.fromPathFile("ClimbRight");
      } catch (Exception e) {
        path = null;
      }
      NamedCommands.registerCommand("ClimbRight", AutoBuilder.pathfindThenFollowPath(path, constraints));

    }

    if (Operating.Constants.USING_INTAKE) {
      intakeSub = new IntakeSubsystem();
      NamedCommands.registerCommand("IntakeToggleRotate", new IntakeToggle(intakeSub));
      NamedCommands.registerCommand("IntakeToggle", new Intake(intakeSub));
    }

    if (Operating.Constants.USING_SHOOTER) {
      shooterSub = new ShooterSubsystem();
      //shooterSub.setDefaultCommand(
          //new RunCommand(() -> shooterSub.updateDesiredRPM(driveSub.getDistanceToHub()), shooterSub));
      NamedCommands.registerCommand("Shoot", new Shoot(shooterSub, 0));
      NamedCommands.registerCommand("Shoot3000", new Shoot(shooterSub, 3000));
    }

    if (Operating.Constants.USING_CLIMB) {
      climbSub = new ClimbSubsystem();
      NamedCommands.registerCommand("ClimbExtend", new ClimbExtend(climbSub));
      NamedCommands.registerCommand("ClimbRetract", new ClimbRetract(climbSub));
      NamedCommands.registerCommand("ClimbZero", new ClimbZero(climbSub));
    }
    // extend if-else chain for other subsystems
  }

  private void configureBindings() {
    int preset = 0;
    switch (preset) {
      default:
        if (Operating.Constants.USING_CLIMB) {
          driverController.povUp().onTrue(new ClimbExtend(climbSub));
          driverController.povDown().onTrue(new ClimbRetract(climbSub));
          driverController.povRight().onTrue(new ClimbZero(climbSub));
        }

        if (Operating.Constants.USING_SHOOTER && Operating.Constants.USING_DRIVE && Operating.Constants.USING_VISION) {
          driverController.leftBumper().onTrue(new ParallelCommandGroup(
              new DriveAlignedToggle(driveSub),
              new Shoot(shooterSub, driveSub.onOpponentSide() ? 3000 : 0)));
        }

        if (Operating.Constants.USING_SHOOTER) {
          driverController.x().onTrue(new Shoot(shooterSub, 2250));
          driverController.rightBumper().onTrue(new Shoot(shooterSub, 3200));
        }

        if (Operating.Constants.USING_INTAKE && Operating.Constants.USING_SHOOTER) {
          driverController.leftTrigger().whileTrue(new FeedToShoot(shooterSub, intakeSub));
        }

        if (Operating.Constants.USING_INTAKE) {
          driverController.b().onTrue(new IntakeToggle(intakeSub));
          driverController.rightTrigger().onTrue(new Intake(intakeSub));
        }

        if(Operating.Constants.USING_OPERATOR && Operating.Constants.USING_SHOOTER) {
          operatorController.x().onTrue(new Shoot(shooterSub, 2000));
          operatorController.y().onTrue(new Shoot(shooterSub, 2150));
          operatorController.b().onTrue(new Shoot(shooterSub, 2300));
          operatorController.a().onTrue(new Shoot(shooterSub, 2450));
          operatorController.povLeft().onTrue(new Shoot(shooterSub, 2600));
          operatorController.povUp().onTrue(new Shoot(shooterSub, 2750));
          operatorController.povRight().onTrue(new Shoot(shooterSub, 2900));
          operatorController.povDown().onTrue(new Shoot(shooterSub, 3050));

        }

        break;
    }
  }

  public Command getAutonomousCommand() {
    if (Operating.Constants.USING_DRIVE) {
      return autoChooser.getSelected();
    } else
      return null;
  }
}
