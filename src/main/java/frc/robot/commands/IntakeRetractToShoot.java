package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetractToShoot extends Command {
    private final IntakeSubsystem intakeSub;

    public IntakeRetractToShoot(IntakeSubsystem intakeSub){
        this.intakeSub = intakeSub;
    }

    @Override 
    public void initialize(){
        intakeSub.retractToShoot();
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
