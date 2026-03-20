package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetract extends Command {
    private final IntakeSubsystem intakeSub;

    public IntakeRetract(IntakeSubsystem intakeSub){
        this.intakeSub = intakeSub;
    }

    @Override 
    public void initialize(){
        intakeSub.retract();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
