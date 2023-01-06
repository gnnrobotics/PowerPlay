package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class setSpecificHeight extends CommandBase {

    // The subsystem the command runs on
    private final LiftSubsystem m_LiftSubsystem;
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;

    public setSpecificHeight(LiftSubsystem subsystem, DoubleSupplier left, DoubleSupplier right) {
        m_LiftSubsystem = subsystem;
        m_left = left;
        m_right = right;
        addRequirements(m_LiftSubsystem);
    }

    @Override
    public void initialize() {
        m_LiftSubsystem.setSpecificHeight(0); // eventually change to set power
    }

    public void execute()
    {
        if (m_left.getAsDouble() == 1 && m_right.getAsDouble() == 1) {
            m_LiftSubsystem.setSpecificHeight(0);
        }
        else if (m_left.getAsDouble() == 1) {
            m_LiftSubsystem.setSpecificHeight(0.5); // change to constant
        }
        else if (m_right.getAsDouble() == 1) {
            m_LiftSubsystem.setSpecificHeight(-0.5); // change to negative of constant
        }
        else {
            m_LiftSubsystem.setSpecificHeight(0);
        }
    }
/* use for lift level code   private enum Level {
        DOWN,
        LOW,
        MEDIUM,
        HIGH
    } */

}