package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private final IntakeSubsystem intakeSub;
    private final double speed;

    public Intake(IntakeSubsystem intakeSub, double speed){
        this.intakeSub = intakeSub;
        this.speed = speed;
    }

    @Override 
    public void initialize(){
        intakeSub.setIntake(speed);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        intakeSub.setIntake(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
