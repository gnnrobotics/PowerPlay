package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
// eventually change to drive teleop or smth

    // The subsystem the command runs on
    private final DriveSubsystem m_DriveSubsystem;
    private final DoubleSupplier m_strafeSpeed;
    private final DoubleSupplier m_forwardSpeed;
    private final DoubleSupplier m_turnSpeed;

    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        m_DriveSubsystem = subsystem;
        m_strafeSpeed = strafeSpeed;
        m_forwardSpeed = forwardSpeed;
        m_turnSpeed = turnSpeed;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.drivePower(0, 0,0, 0);
    }

    public void execute()
    {
        double forwardSpeed = m_forwardSpeed.getAsDouble();
        double strafeSpeed = m_strafeSpeed.getAsDouble();
        double turnSpeed = m_turnSpeed.getAsDouble();

        double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (forwardSpeed + strafeSpeed + turnSpeed) / denominator;
        double frontRightPower = (forwardSpeed - strafeSpeed - turnSpeed) / denominator;
        double backLeftPower = (forwardSpeed - strafeSpeed + turnSpeed) / denominator;
        double backRightPower = (forwardSpeed + strafeSpeed - turnSpeed) / denominator;


        m_DriveSubsystem.drivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

}