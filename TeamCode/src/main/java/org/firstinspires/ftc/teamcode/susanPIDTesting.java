package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.commands.liftPIDCommand;
import org.firstinspires.ftc.teamcode.robot.commands.susanPIDCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

@Config
@TeleOp(name = "susanPID")

public class susanPIDTesting extends CommandOpMode {

    private SusanSubsystem m_susan;
    private susanPIDCommand m_susanPID;
    public static double p = 0, i = 0, d = 0;
    public static double target = 50;

    @Override
    public void initialize() {
        // organize by subsystems then commands

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m_susan = new SusanSubsystem(hardwareMap, "Susan");

        m_susanPID = new susanPIDCommand(m_susan, () -> target, () -> p, () -> i, () -> d);
        register(m_susan);
        System.out.println(target);
        m_susan.setDefaultCommand(m_susanPID);
    }
    public void run() {
        super.run();
        telemetry.addData("pos", m_susan.getEncoder());
        telemetry.addData("target", target);
        telemetry.addData("power", m_susan.getPower());
        telemetry.update();
    }
}