package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb;
import frc.robot.Constants.Configs;
import frc.robot.Constants.IDs;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkMax;

public class ClimbSubsystem extends SubsystemBase{
    
    private final PIDMotor innerLeft;
    private final PIDMotor innerRight;
    
    //private final SparkMax outerLeft;
    //private final SparkMax outerRight;

    //add limit switches?
    public ClimbSubsystem() {
        innerLeft = new PIDMotor(new PIDMotorIOSparkMax(IDs.ClimbConstants.CLIMB_LEFT_ID, Configs.Climb.INNER_LEFT_CONFIG));
        innerRight = new PIDMotor(new PIDMotorIOSparkMax(IDs.ClimbConstants.CLIMB_RIGHT_ID, Configs.Climb.INNER_RIGHT_CONFIG));
        resetEncoders();
    }

    public void moveLeft(double speed) {
        innerLeft.set(speed);
    }

    public void moveRight(double speed) {
        innerRight.set(speed);
    }

    public void moveInner(double speed) {
        moveLeft(speed);
        moveRight(speed);
    }

    public void raise() {
        while(innerLeft.getEncoder() > -317)
            moveInner(-0.8);
        moveInner(0);
    }

    public void retract() {
        while(innerLeft.getEncoder() < 0)
            moveInner(0.8);
        moveInner(0);
    }

    public void moveOuter(double speed) {
        //if bot-limit switch
        //speed = Math.max(speed, 0) <- prevents motors from impossibly going down
        //outerLeft.set(speed);
    }

    public void resetEncoders() {
        innerLeft.resetEncoder();
        innerRight.resetEncoder();
    }

    public void stop() {
        innerLeft.stopMotors();
        innerRight.stopMotors();
    }

    public double getEncoder() {
        return innerLeft.getEncoder();
    }

    @Override
    public void periodic () {
        Logger.recordOutput("Climb/Left", innerLeft.getEncoder());
        Logger.recordOutput("Climb/Right", innerRight.getEncoder());
    }
}
