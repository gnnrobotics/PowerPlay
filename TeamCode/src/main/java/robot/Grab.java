package robot;

import com.arcrobotics.ftclib.command.CommandBase;


/**
 * A simple command that grabs a stone with the {@link GripperSubsystem}.  Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * com.arcrobotics.ftclib.command.InstantCommand}.
 */
public class Grab extends CommandBase {

    // The subsystem the command runs on
    private final ClawSubsystem m_ClawSubsystem;

    public Grab(ClawSubsystem subsystem) {
        m_ClawSubsystem = subsystem;
        addRequirements(m_ClawSubsystem);
    }

    @Override
    public void initialize() {
        m_ClawSubsystem.grab();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}