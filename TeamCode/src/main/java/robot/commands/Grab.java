package robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import robot.subsystems.ClawSubsystem;


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