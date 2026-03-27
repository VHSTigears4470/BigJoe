package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private final IntakeSubsystem intakeSub;

    public Intake(IntakeSubsystem intakeSub){
        this.intakeSub = intakeSub;
    }

    @Override 
    public void initialize(){
        intakeSub.toggleIntake();
        if(intakeSub.isRunning())
            intakeSub.setIntake(0.8);
        else
            intakeSub.setIntake(0);
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
