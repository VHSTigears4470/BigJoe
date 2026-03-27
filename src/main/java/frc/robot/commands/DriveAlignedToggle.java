package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAlignedToggle extends Command {
    private final DriveSubsystem driveSub;

    public DriveAlignedToggle(DriveSubsystem driveSub){
        this.driveSub = driveSub;
    }

    @Override 
    public void initialize(){
        driveSub.toggleDrivingAligned();
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
