package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class OneMotorCommand extends CommandBase {

    // The subsystem the command runs on
    private final DriveSubsystem m_DriveSubsystem;
    private final String whichMotor;

    public OneMotorCommand(DriveSubsystem subsystem, String motor) {
        m_DriveSubsystem = subsystem;
        whichMotor = motor;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.drivePower(0, 0,0, 0);
    }

    public void execute()
    {
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double backLeftPower = 0;
        double backRightPower = 0;

        if(whichMotor.equals("fl")){
             frontRightPower = 0;
             backLeftPower = 0;
             backRightPower = 0;
            frontLeftPower = 0.1;
        }
        else if(whichMotor.equals("fr")){
            frontRightPower = 0.1;
            backLeftPower = 0;
            backRightPower = 0;
            frontLeftPower = 0;
        }
        else if(whichMotor.equals("bl")){
            frontRightPower = 0;
            backLeftPower = 0.1;
            backRightPower = 0;
            frontLeftPower = 0;
        }
        else if(whichMotor.equals("br")){
            frontRightPower = 0;
            backLeftPower = 0;
            backRightPower = 0.1;
            frontLeftPower = 0;
        }
        m_DriveSubsystem.drivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

}