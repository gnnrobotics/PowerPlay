package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import robot.subsystems.ClawSubsystem;
import robot.commands.Grab;
import robot.commands.Release;

@TeleOp(name = "TestingClaw")

public class TestingOpMode extends CommandOpMode {

    private GamepadEx m_driverOp;
    private ClawSubsystem m_claw;
    private Grab m_grabCommand;
    private Release m_releaseCommand;
    Button m_toggleButton;

    @Override
    public void initialize() {

        m_driverOp = new GamepadEx(gamepad1);

        m_claw = new ClawSubsystem(hardwareMap, "Claw");
        m_grabCommand = new Grab(m_claw);
        m_releaseCommand = new Release(m_claw);

        m_toggleButton = new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(m_grabCommand, m_releaseCommand);

       // m_claw.setDefaultCommand(m_grabCommand); // this is necessary to state the default state
    }


}