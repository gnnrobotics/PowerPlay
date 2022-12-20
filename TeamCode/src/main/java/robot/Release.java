package robot;

import com.arcrobotics.ftclib.command.CommandBase;

public class Release extends CommandBase {

    // The subsystem the command runs on
    private final ClawSubsystem m_ClawSubsystem;

    public Release(ClawSubsystem subsystem) {
        m_ClawSubsystem = subsystem;
        addRequirements(m_ClawSubsystem);
    }

    @Override
    public void initialize() {
        m_ClawSubsystem.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}