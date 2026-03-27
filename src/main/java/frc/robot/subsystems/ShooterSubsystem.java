package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs.Shooter;
import frc.robot.Constants.IDs.ShooterConstants;
import frc.robot.components.PIDMotor;
import frc.robot.components.PIDMotorIOSparkFlex;
import frc.robot.components.PIDMotorIOSparkMax;

public class ShooterSubsystem extends SubsystemBase{
    
    private final PIDMotor flywheelRight; //leader
    private final PIDMotor flywheelLeft;
    private final PIDMotor feeder; 
    private final PIDMotor hopper;
    private final InterpolatingDoubleTreeMap table;
    private double RPMToHub;
    private double desiredRPM;
    private boolean shooterActive;

    public ShooterSubsystem() {
        flywheelRight = new PIDMotor(new PIDMotorIOSparkFlex(ShooterConstants.FLYWHEEL_RIGHT_ID, Shooter.FLYWHEEL_RIGHT_CONFIG));
        flywheelLeft = new PIDMotor(new PIDMotorIOSparkFlex(ShooterConstants.FLYWHEEL_LEFT_ID, Shooter.FLYWHEEL_LEFT_CONFIG));
        feeder = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.FEEDER_ID, Shooter.FEEDER_CONFIG));
        hopper = new PIDMotor(new PIDMotorIOSparkMax(ShooterConstants.HOPPER_ID, Shooter.HOPPER_CONFIG));
        
        table = new InterpolatingDoubleTreeMap();
        table.put(2.95, 2750.0); 
        table.put(3.45, 2950.0); 
        table.put(Units.inchesToMeters(177.75), 5000.0); 

        RPMToHub = 0;
        desiredRPM = 0;
        shooterActive = false;
    }

    public void setDesiredRPM(double desiredRPM) {
        this.desiredRPM = desiredRPM;
        SmartDashboard.putNumber("Desired RPM", desiredRPM);
        Logger.recordOutput("Shooter/Desired RPM", desiredRPM);
    }

    public void toggleShooter() {
        shooterActive = !shooterActive;
        if(!shooterActive) {
            desiredRPM = 0;
        }
    }

    public boolean IsShooting() {
        return shooterActive;
    }

    public boolean flywheelReady() {
        return (desiredRPM != 0) 
            ? Math.abs(flywheelRight.getRPM() - desiredRPM)  < desiredRPM * 0.15 
            : Math.abs(flywheelRight.getRPM()- RPMToHub)  < RPMToHub * 0.15;
    }
    
    public void updateDesiredRPM(double distance) {
        RPMToHub = table.get(distance);
        Logger.recordOutput("Shooter/Aligned RPM", RPMToHub);
    }

    public void setFeeder(double speed) {
        feeder.set(speed);
        Logger.recordOutput("Shooter/Feeder", speed);
    }

    public void setHopper(double speed) {
        hopper.set(speed);
        Logger.recordOutput("Shooter/Hopper", speed);
    }

    public void stopMotors() {
        flywheelRight.stopMotors();
        flywheelLeft.stopMotors();
        feeder.stopMotors();
        hopper.stopMotors();
    }

    @Override
    public void periodic() {
        if(!shooterActive) {
            if(Math.abs(flywheelRight.getRPM()) < 200) {
                flywheelRight.set(0);
            } else {
                flywheelRight.setVelocity(0, 0.00020352); 
            }
        } else if(desiredRPM != 0){
            flywheelRight.setVelocity(desiredRPM, 0.00020352);
        } else {
            flywheelRight.setVelocity(RPMToHub, 0.00020352); 
        }
        if(shooterActive && flywheelReady()) {
            setFeeder(0.95);
            setHopper(0.5);
        } else {
            setFeeder(0);
            setHopper(0);
        }
        SmartDashboard.putNumber("Actual RPM", flywheelRight.getRPM());
        Logger.recordOutput("RPM/Actual", flywheelRight.getRPM());
    }
}
