package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbZero extends Command{
    private final ClimbSubsystem climbSub;

    public ClimbZero(ClimbSubsystem climbSub){
        this.climbSub = climbSub;
        addRequirements(climbSub);
    }

    @Override 
    public void initialize(){
        if(climbSub.getEncoder() > 0)
            climbSub.move(-0.8);
        else 
            climbSub.move(0.8);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){
        climbSub.move(0);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(climbSub.getEncoder()) < 3;
    }
}
