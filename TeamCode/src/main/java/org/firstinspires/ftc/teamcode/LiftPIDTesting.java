package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

    Button m_toggleButton;

    private LiftSubsystem m_lift;
    private liftPIDCommand m_liftPID;
    private GamepadEx m_driverOp;
    private Grab m_grabCommand;
    private Release m_releaseCommand;

    public static double target = 200;
    private ClawSubsystem m_claw;

    @Override
    public void initialize() {
        // organize by subsystems then commands

        m_driverOp = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m_lift = new LiftSubsystem(hardwareMap, "Lift");
        m_claw = new ClawSubsystem(hardwareMap);
        m_liftPID = new liftPIDCommand(m_lift, () -> target);

        m_claw = new ClawSubsystem(hardwareMap);
        m_grabCommand = new Grab(m_claw);
        m_releaseCommand = new Release(m_claw);

        m_toggleButton = new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(m_grabCommand, m_releaseCommand);

        register(m_lift);
        register(m_claw);

        m_lift.setDefaultCommand(m_liftPID);
    }
    public void run() {
        //vroom vroom
        super.run();
        telemetry.addData("pos", m_lift.getEncoder());
        telemetry.addData("target", target);
        telemetry.addData("power", m_lift.getPower());
        telemetry.update();
    }
}