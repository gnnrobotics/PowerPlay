package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCopy extends CommandBase {
// eventually change to drive teleop or smth

    // The subsystem the command runs on
    private final DriveSubsystem m_DriveSubsystem;
    private final DoubleSupplier m_strafeSpeed;
    private final DoubleSupplier m_forwardSpeed;
    private final DoubleSupplier m_turnSpeed;

    public DriveCopy(DriveSubsystem subsystem, DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        m_DriveSubsystem = subsystem;
        m_strafeSpeed = strafeSpeed;
        m_forwardSpeed = forwardSpeed;
        m_turnSpeed = turnSpeed;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
       //  m_DriveSubsystem.drive(0, 0, 0);
    }

    public void execute()
    {
      //  m_DriveSubsystem.drive(m_strafeSpeed.getAsDouble(), m_forwardSpeed.getAsDouble(), m_turnSpeed.getAsDouble());
    }

}