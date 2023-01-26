package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
@Config
@TeleOp(name = "LiftPID")

public class LiftPIDTesting extends CommandOpMode {

    private LiftSubsystem m_lift;
    private liftPIDCommand m_liftPID;
    private static double p = 0, i = 0, d = 0, f = 0;
    private int target = 0;

    @Override
    public void initialize() {
        // organize by subsystems then commands

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m_lift = new LiftSubsystem(hardwareMap, "Lift");

        m_liftPID = new liftPIDCommand(m_lift, () -> target, () -> p, () -> i, () -> d, () -> f);
        register(m_lift);

        m_lift.setDefaultCommand(m_liftPID);
    }
    public void run() {
        telemetry.addData("pos", m_lift.getEncoder());
        telemetry.addData("target", target);
        telemetry.update();
    }


}