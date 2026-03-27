package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs;
import frc.robot.Constants.IDs;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkMax;

public class ClimbSubsystem extends SubsystemBase{
    
    private final PIDMotor inner; 

    public ClimbSubsystem() {
        inner = new PIDMotor(new PIDMotorIOSparkMax(IDs.ClimbConstants.CLIMB_ID, Configs.Climb.LEFT_CONFIG));
        resetEncoders();
    }
    
    public void move(double speed) {
        inner.set(speed);
    }

    public void resetEncoders() {
        inner.resetEncoder();
    }

    public void stop() {
        inner.stopMotors();
    }

    public double getEncoder() {
        return inner.getEncoder();
    }

    @Override
    public void periodic () {
        Logger.recordOutput("Climb/Encoder", inner.getEncoder());
    }
}
