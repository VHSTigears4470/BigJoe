package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.IntakeConstants;


public class KitbotIntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    public KitbotIntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushed);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(50);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putNumber("Intake Feed Voltage", IntakeConstants.INTAKE_FEED_VOLT);
    }

    public void setFeederRoller(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void intake() {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_FEED_VOLT);
    }

    public void reverse() {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_REVERSE_VOLT);
    }

    public void stopIntake() {
        intakeMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {}
}
