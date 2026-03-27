package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem shooterSub;
    private final double rpm;

    public Shoot(ShooterSubsystem shooterSub, double rpm){
        this.shooterSub = shooterSub;
        this.rpm = rpm;
        addRequirements(shooterSub);
    }

    @Override 
    public void initialize(){
        shooterSub.setDesiredRPM(rpm);
        shooterSub.toggleShooter();
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
