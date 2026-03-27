package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedToShoot extends Command {
    private final ShooterSubsystem shooterSub;
    private final IntakeSubsystem intakeSub;
    
    public FeedToShoot(ShooterSubsystem shooterSub, IntakeSubsystem intakeSub){
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;
    }

    @Override 
    public void initialize(){
        shooterSub.setFeeder(0.95);
        shooterSub.setHopper(0.5);
        intakeSub.setIntake(0.8);
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        shooterSub.setFeeder(0);
        shooterSub.setHopper(0);
        intakeSub.setIntake(0);
        intakeSub.extend();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
