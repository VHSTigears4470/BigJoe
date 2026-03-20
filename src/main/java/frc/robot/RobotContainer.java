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

  private final CommandXboxController controller =
      new CommandXboxController(OI.Constants.DRIVE_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    initSubystems();
    
     PathConstraints constraints = new PathConstraints(
                1.5, 2.0,
                Units.degreesToRadians(240), 
                Units.degreesToRadians(240));
     PathPlannerPath path;
      try{
        path = PathPlannerPath.fromPathFile("ToFrame");
      } catch (Exception e) {
        path = null;
      }       
      NamedCommands.registerCommand("moveToFrame", AutoBuilder.pathfindThenFollowPath(path, constraints));

      try{
        path = PathPlannerPath.fromPathFile("ShootRight");
      } catch (Exception e) {
        path = null;
      }       
      NamedCommands.registerCommand("ShootRight", AutoBuilder.pathfindThenFollowPath(path, constraints));

      try{
        path = PathPlannerPath.fromPathFile("ShootCenter");
      } catch (Exception e) {
        path = null;
      }       
      NamedCommands.registerCommand("ShootCenter", AutoBuilder.pathfindThenFollowPath(path, constraints));

      try{
        path = PathPlannerPath.fromPathFile("ShootLeft");
      } catch (Exception e) {
        path = null;
      }       
      NamedCommands.registerCommand("ShootLeft", AutoBuilder.pathfindThenFollowPath(path, constraints));
    
    if(Operating.Constants.USING_DRIVE){
      autoChooser = AutoBuilder.buildAutoChooser();
      //Add paths here
      autoChooser.addOption("1", new PathPlannerAuto("1"));      
      autoChooser.addOption("ShootCenter", new PathPlannerAuto("ShootCenter")); 
      autoChooser.addOption("ShootLeft", new PathPlannerAuto("ShootLeft"));
      autoChooser.addOption("ShootRight", new PathPlannerAuto("ShootRight")); 
      autoChooser.addOption("Collect Frame", new PathPlannerAuto("Collect Frame")); 
      SmartDashboard.putData("Auto Mode", autoChooser);
    } else {
      autoChooser = null;
    }

    configureBindings();

    FollowPathCommand.warmupCommand().schedule();
  }

  public void initSubystems() {
    if(Operating.Constants.USING_VISION) {
      visionSub = new VisionSubsystem();
    }

    if(Operating.Constants.USING_DRIVE) {
      driveSub = new DriveSubsystem(Optional.ofNullable(visionSub));
      driveSub.setDefaultCommand(new RunCommand(
        () -> {
          double y = OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND);
          double x = OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND);
          double rot = OI.Constants.DRIVER_AXIS_ROT_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_ROT), OI.Constants.DRIVE_DEADBAND);

          //Add logging for buttons

          // Record operator inputs with the project logger
          Logger.recordOutput("Operator/Drive/Y", y);
          Logger.recordOutput("Operator/Drive/X", x);
          Logger.recordOutput("Operator/Drive/Rot", rot);
          Logger.recordOutput("Operator/Drive/LeftTrigger", controller.leftTrigger().getAsBoolean());
          Logger.recordOutput("Operator/Drive/RightTrigger", controller.rightTrigger().getAsBoolean());

          driveSub.drive(y, x, rot, true, "Default / Field Oriented"); 
        },
        driveSub));

        NamedCommands.registerCommand("DriveAligned", new RunCommand(() -> driveSub.driveAligned(
                  OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND),
                  true,
                  "Aiming / Field Oriented"
            ), 
            driveSub));
    }

    if(Operating.Constants.USING_INTAKE) {
      intakeSub = new IntakeSubsystem();
      NamedCommands.registerCommand("Extend", new IntakeExtend(intakeSub));
      NamedCommands.registerCommand("Intake", new Intake(intakeSub, 1));
      NamedCommands.registerCommand("RaiseToShoot", new IntakeRetractToShoot(intakeSub));
    }

    if(Operating.Constants.USING_SHOOTER) {
      shooterSub = new ShooterSubsystem();
      shooterSub.setDefaultCommand(new RunCommand(() -> shooterSub.updateDesiredRPM(driveSub.getDistanceToHub()), shooterSub));
      NamedCommands.registerCommand("Shoot", new Shoot(shooterSub,0));
      if(Operating.Constants.USING_INTAKE)
        NamedCommands.registerCommand("FeedToShoot", new FeedToShoot(shooterSub, intakeSub));
    }

    if(Operating.Constants.USING_CLIMB) {
      climbSub = new ClimbSubsystem(); 
      climbSub.setDefaultCommand(new RunCommand(() -> climbSub.move(0), climbSub));
    } 

    

    // extend if-else chain for other subsystems
  }

  private void configureBindings() {
  int preset = 0;
    switch (preset) {
      default:
        if(Operating.Constants.USING_CLIMB) {
          controller.y().whileTrue(new RunCommand(() -> climbSub.move(0.3), climbSub));
          controller.a().whileTrue(new RunCommand(() -> climbSub.move(-0.3), climbSub));
        }

        if(Operating.Constants.USING_SHOOTER && Operating.Constants.USING_DRIVE && Operating.Constants.USING_VISION) {
          controller.rightTrigger().whileTrue(new ParallelCommandGroup(
            new RunCommand(() -> driveSub.driveAligned(
                  OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND),
                  true,
                  "Aiming / Field Oriented"
            ), 
            driveSub),
            new Shoot(shooterSub,0)
          ));
        }

        if(Operating.Constants.USING_SHOOTER) {
          controller.a().whileTrue(new Shoot(shooterSub, 5000));
        }
        
        if(Operating.Constants.USING_INTAKE && Operating.Constants.USING_SHOOTER){
          controller.rightBumper().whileTrue(new FeedToShoot(shooterSub, intakeSub));
        }

        if(Operating.Constants.USING_INTAKE) {
          controller.x().onTrue(new RunCommand(() -> intakeSub.extend(), intakeSub));
          controller.y().onTrue(new RunCommand(() -> intakeSub.retract(), intakeSub));
          controller.b().whileTrue(new Intake(intakeSub, 0.7));
        }
        
        break;
    }    
}

  public Command getAutonomousCommand() {
    if(Operating.Constants.USING_DRIVE){
      return autoChooser.getSelected();
    }
    else 
      return null;
  }
}
