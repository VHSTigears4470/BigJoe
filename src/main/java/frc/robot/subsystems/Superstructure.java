package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.States.GameMode;
import frc.robot.Constants.States.RobotMode;

public class Superstructure {
    private final IntakeSubsystem intakeSubsystem;
    private RobotMode currentMode;
    private RobotMode targetMode;
    private GameMode gameMode;
    private boolean hubActive;

    public Superstructure(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        hubActive = false;
        currentMode = RobotMode.IDLE;
        targetMode = RobotMode.IDLE;
        gameMode = GameMode.INITIAL;
    }

    public void setTargetMode(RobotMode mode) {
        targetMode = mode;
    }

    public boolean atTargetMode(){
        return currentMode == targetMode;
    }

    public RobotMode getCurrentMode() {
        return currentMode;
    }

    public RobotMode getTargetMode() {
        return targetMode;
    }

    public void updateGameState(boolean autoWon) {
        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous())
            gameMode = GameMode.AUTONOMOUS;
        else if (matchTime >= 130)
            gameMode = GameMode.TRANSITION;
        else if (matchTime >= 105)
            gameMode = GameMode.SHIFT_1;
        else if (matchTime >= 80)
            gameMode = GameMode.SHIFT_2;
        else if (matchTime >= 55)
            gameMode = GameMode.SHIFT_3;
        else if (matchTime >= 30)
            gameMode = GameMode.SHIFT_4;
        else
            gameMode = GameMode.ENDGAME;

        if (autoWon && (gameMode == GameMode.SHIFT_1 || gameMode == GameMode.SHIFT_3))
            hubActive = false;
        else
            hubActive = true;
    }

    public boolean isHubActive() {
        return hubActive;
    }

    public void periodic() {
        switch(currentMode){
            case IDLE:
                // IDLE
                break; 
            case INTAKING_PREP:
                // INTAKING_PREP
                break;
            case INTAKING:
                // INTAKING
                break;
            case SHOOTING_PREP:
                // SHOOTING_PREP
                break;
            case SHOOTING:
                // SHOOTING
                break;
        }
    }

    public void handleIdleMode() {
        intakeSubsystem.stopMotors();
    }

    public void handleIntakingPrepMode() {
        
        // intakeSubsystem.setIntakeRPM(0); ask
        // intakeSubsystem.setRotateRPMForward(0); ask
    }

    public void handleIntakingMode() {
        double goal = 0.0;
        // intakeSubsystem.setIntakeRPM(0); ask
        // intakeSubsystem.setRotateRPMForward(0); ask
    }

    public void handleShootingPrepMode() {
        
    }

    public void handleShootingMode() {
        
    }
}
