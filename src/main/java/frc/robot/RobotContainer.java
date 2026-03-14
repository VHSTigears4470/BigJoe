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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem driveSub;
  private VisionSubsystem visionSub;
  private ShooterSubsystem shooterSub;
  private ClimbSubsystem climbSub;

  private final CommandXboxController controller =
      new CommandXboxController(OI.Constants.DRIVE_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initSubystems();
    /*
     PathConstraints constraints = new PathConstraints(
                1.5, 2.0,
                Units.degreesToRadians(240), 
                Units.degreesToRadians(240));

      PathPlannerPath path;
      try {
        //path = PathPlannerPath.fromPathFile("Climb");
      } catch (Exception e) {
        path = null;
      }

      //NamedCommands.registerCommand("moveToClimb", AutoBuilder.pathfindThenFollowPath(path, constraints));
      NamedCommands.registerCommand("ClimbRetract", new ClimbRetract(climbSub));
      NamedCommands.registerCommand("ClimbExtend", new ClimbExtend(climbSub));
    */
    if(Operating.Constants.USING_DRIVE){
      autoChooser = AutoBuilder.buildAutoChooser();
      //Add paths here
      autoChooser.addOption("1", new PathPlannerAuto("1"));      
      autoChooser.addOption("2", new PathPlannerAuto("2")); 
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
    }

    if(Operating.Constants.USING_SHOOTER) {
      shooterSub = new ShooterSubsystem();
      shooterSub.setDefaultCommand(new RunCommand(
        () -> {
          Logger.recordOutput("Operator/Shooter/RightTrigger", controller.rightTrigger().getAsBoolean());
          shooterSub.setRPM(0);
        }, shooterSub));
    }

    if(Operating.Constants.USING_CLIMB) {
      climbSub = new ClimbSubsystem(); 
      climbSub.setDefaultCommand(new RunCommand(() -> climbSub.moveInner(0), climbSub));
    } 

    // extend if-else chain for other subsystems
  }

  private void configureBindings() {
  int preset = 0;
    switch (preset) {
      default:
        if(Operating.Constants.USING_VISION) {
            controller.leftTrigger().whileTrue(new RunCommand(() -> driveSub.driveAligned(
                  OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND),
                  true,
                  "Aiming / Field Oriented"
            ), 
            driveSub));
        }

        if(Operating.Constants.USING_CLIMB) {
          controller.y().whileTrue(new RunCommand(() -> climbSub.moveLeft(-0.8), climbSub));
          controller.x().whileTrue(new RunCommand(() -> climbSub.moveLeft(0.8), climbSub));
          controller.a().whileTrue(new RunCommand(() -> climbSub.moveRight(-0.8), climbSub));
          controller.b().whileTrue(new RunCommand(() -> climbSub.moveRight(0.8), climbSub));
        }

        if(Operating.Constants.USING_SHOOTER) {
          controller.rightTrigger().whileTrue(new RunCommand(() -> shooterSub.setRPM(-2500), shooterSub));
          controller.rightBumper().whileTrue(new RunCommand(() -> shooterSub.setSecondary(-0.8), shooterSub));
          controller.rightBumper().whileFalse(new RunCommand(() -> shooterSub.setSecondary(0), shooterSub));
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
