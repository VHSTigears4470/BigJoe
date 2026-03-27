package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs.Intake;
import frc.robot.Constants.IDs.IntakeConstants;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkMax;

public class IntakeSubsystem extends SubsystemBase {
    
    private final PIDMotor intakeMotor;
    private final PIDMotor rotateMotor;
    private boolean isRetracted;
    private boolean isRunning;

    public IntakeSubsystem(){
        intakeMotor = new PIDMotor(new PIDMotorIOSparkMax(IntakeConstants.INTAKE_ID, Intake.INTAKE_CONFIG));
        rotateMotor = new PIDMotor(new PIDMotorIOSparkMax(IntakeConstants.ROTATE_ID, Intake.ROTATE_CONFIG));
        isRetracted = true;
        isRunning = false;
    }

    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void toggleIntake() {
        isRunning = !isRunning;
    }
    
    public boolean isRunning() {
        return isRunning;
    }

    public void extend() {
        rotateMotor.setSetpoint(17, 0);
    }

    public void retract() {
        rotateMotor.setSetpoint(-3.0, 0);
    }

    public void toggleRotate() {
        isRetracted = !isRetracted;
    }

    public boolean isRetracted(){
        return isRetracted;
    }

    public void stopMotors() {
        intakeMotor.stopMotors();
        rotateMotor.stopMotors();
    }

    public double getRotation() {
        return rotateMotor.getEncoder();
    }

    public void periodic(){
        Logger.recordOutput("IntakeSubsystem/Is Retracted", isRetracted);
        Logger.recordOutput("IntakeSubsystem/Rotate Value", getRotation());
        Logger.recordOutput("IntakeSubsystem/IntakeMotorRPM", intakeMotor.getRPM());
        Logger.recordOutput("IntakeSubsystem/RotateMotorRPM", rotateMotor.getRPM());
    }
}
