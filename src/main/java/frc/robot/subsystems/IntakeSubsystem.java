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

    public IntakeSubsystem(){
        intakeMotor = new PIDMotor(new PIDMotorIOSparkMax(IntakeConstants.INTAKE_ID, Intake.INTAKE_CONFIG));
        rotateMotor = new PIDMotor(new PIDMotorIOSparkMax(IntakeConstants.ROTATE_ID, Intake.ROTATE_CONFIG));
    }

    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void extend() {
        rotateMotor.setSetpoint(17, 0);
    }

    public void retract() {
        rotateMotor.setSetpoint(-1.0, 0);
    }

    public void retractToShoot() {
        rotateMotor.setSetpoint(1.5, 0);
    }

    public void stopMotors() {
        intakeMotor.stopMotors();
        rotateMotor.stopMotors();
    }

    public double getRotation() {
        return rotateMotor.getEncoder();
    }

    public void periodic(){
        Logger.recordOutput("IntakeSubsystem/Rotate Value", rotateMotor.getEncoder());
        Logger.recordOutput("IntakeSubsystem/IntakeMotorRPM", intakeMotor.getRPM());
        Logger.recordOutput("IntakeSubsystem/RotateMotorRPM", rotateMotor.getRPM());
    }
}
