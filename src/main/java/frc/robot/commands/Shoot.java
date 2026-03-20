package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem shooterSub;
    private final double rpm;

    public Shoot(ShooterSubsystem shooterSub, double rpm){
        this.shooterSub = shooterSub;
        this.rpm = rpm;
    }

    @Override 
    public void initialize(){
        shooterSub.toggleShooter();
        if(rpm != 0)
            shooterSub.setDesiredRPM(rpm);

    }

    @Override
    public void execute(){
        if(rpm != 0)
            shooterSub.setDesiredRPM(rpm);
    }

    @Override
    public void end(boolean interrupted){
        shooterSub.toggleShooter();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
