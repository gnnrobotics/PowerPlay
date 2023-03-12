package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.susanSpeed;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

import java.util.function.BooleanSupplier;

public class spinSusan extends CommandBase {

    // The subsystem the command runs on
    private final SusanSubsystem m_SusanSubsystem;
    private final BooleanSupplier m_left;
    private final BooleanSupplier m_right;
    private final BooleanSupplier m_coOpleft;
    private final BooleanSupplier m_coOpright;

    public spinSusan(SusanSubsystem subsystem, BooleanSupplier left, BooleanSupplier right, BooleanSupplier coOpLeft, BooleanSupplier coOpRight) {
        m_SusanSubsystem = subsystem;
        m_left = left;
        m_right = right;
        m_coOpleft = coOpLeft;
        m_coOpright = coOpRight;
        addRequirements(m_SusanSubsystem);
    }

    @Override
    public void initialize() {
        m_SusanSubsystem.spinSusan(0);
    }

    public void execute()
    {
        if((m_coOpleft.getAsBoolean() || m_coOpright.getAsBoolean()))
        {
            if (m_coOpleft.getAsBoolean() && m_coOpright.getAsBoolean()) {
                m_SusanSubsystem.spinSusan(0);
            }
            else if (m_coOpleft.getAsBoolean()) {
                m_SusanSubsystem.spinSusan(susanSpeed); // change to constant
            }
            else if (m_coOpright.getAsBoolean()) {
                m_SusanSubsystem.spinSusan(-susanSpeed); // change to negative of constant
            }
            else {
                m_SusanSubsystem.spinSusan(0);
            }
        }
        else {
            if (m_left.getAsBoolean() && m_right.getAsBoolean()) {
                m_SusanSubsystem.spinSusan(0);
            }
            else if (m_left.getAsBoolean()) {
                m_SusanSubsystem.spinSusan(susanSpeed); // change to constant
            }
            else if (m_right.getAsBoolean()) {
                m_SusanSubsystem.spinSusan(-susanSpeed); // change to negative of constant
            }
            else {
                m_SusanSubsystem.spinSusan(0);
            }
        }

    }
}