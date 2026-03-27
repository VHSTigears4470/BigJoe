package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToggle extends Command {
    private final IntakeSubsystem intakeSub;

    public IntakeToggle(IntakeSubsystem intakeSub){
        this.intakeSub = intakeSub;
    }

    @Override 
    public void initialize(){
        intakeSub.toggleRotate();
        if(intakeSub.isRetracted())
            intakeSub.retract();
        else
            intakeSub.extend();
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
