package robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import robot.subsystems.LiftSubsystem;

public class setLevel extends CommandBase {

    // The subsystem the command runs on
    private final LiftSubsystem m_LiftSubsystem;
    private Level level;

    public setLevel(LiftSubsystem subsystem, Level levelInput) {
        m_LiftSubsystem = subsystem;
        level = levelInput;
        addRequirements(m_LiftSubsystem);
    }

    @Override
    public void initialize() {
        switch(level) {
            case DOWN:
                break;
            case LOW:
                break;
            case MEDIUM:
                break;
            case HIGH:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    private enum Level {
        DOWN,
        LOW,
        MEDIUM,
        HIGH
    }

}