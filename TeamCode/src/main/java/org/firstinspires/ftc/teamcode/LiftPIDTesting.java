package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.commands.FineDriveCommand;
import org.firstinspires.ftc.teamcode.robot.commands.Grab;
import org.firstinspires.ftc.teamcode.robot.commands.Release;
import org.firstinspires.ftc.teamcode.robot.commands.liftPIDCommand;
import org.firstinspires.ftc.teamcode.robot.commands.setSpecificHeight;
import org.firstinspires.ftc.teamcode.robot.commands.spinSusan;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

@TeleOp(name = "LiftPID")

public class LiftPIDTesting extends CommandOpMode {

    private LiftSubsystem m_lift;
    private liftPIDCommand m_liftPID;


    @Override
    public void initialize() {
// organize by subsystems then commands


        m_lift = new LiftSubsystem(hardwareMap, "Lift");
        m_liftPID = new liftPIDCommand(m_lift, () -> 500);

        register(m_lift);

        m_lift.setDefaultCommand(m_liftPID);
    }


}