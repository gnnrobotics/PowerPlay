package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import robot.ClawSubsystem;
import robot.Grab;
import robot.Release;

@TeleOp(name = "TestingClaw")

public class TestingOpMode extends CommandOpMode {

    private GamepadEx m_driverOp;
    private ClawSubsystem m_claw;
    private Grab m_grabCommand;
    private Release m_releaseCommand;
    private Button m_grabButton, m_releaseButton;

    @Override
    public void initialize() {

        m_driverOp = new GamepadEx(gamepad1);

        m_claw = new ClawSubsystem(hardwareMap, "Claw");
        m_grabCommand = new Grab(m_claw);
        m_releaseCommand = new Release(m_claw);

        m_grabButton = (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
                .whenPressed(m_grabCommand);
        m_releaseButton = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
                .whenPressed(m_releaseCommand);
    }

}