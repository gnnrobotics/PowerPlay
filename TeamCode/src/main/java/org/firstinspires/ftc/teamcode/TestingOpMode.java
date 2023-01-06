package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.commands.setSpecificHeight;
import org.firstinspires.ftc.teamcode.robot.commands.spinSusan;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.commands.Grab;
import org.firstinspires.ftc.teamcode.robot.commands.Release;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

@TeleOp(name = "TestingClaw")

public class TestingOpMode extends CommandOpMode {

    private GamepadEx m_driverOp;

    private ClawSubsystem m_claw;;
    private Grab m_grabCommand;
    private Release m_releaseCommand;

    private LiftSubsystem m_lift;
    private setSpecificHeight m_heightCommand;

    private SusanSubsystem m_susan;
    private spinSusan m_spinCommand;

    Button m_toggleButton;

    @Override
    public void initialize() {

        m_driverOp = new GamepadEx(gamepad1);

        m_claw = new ClawSubsystem(hardwareMap, "Claw");
        m_grabCommand = new Grab(m_claw);
        m_releaseCommand = new Release(m_claw);

        m_lift = new LiftSubsystem(hardwareMap, "Lift");
        m_heightCommand = new setSpecificHeight(m_lift, () -> m_driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), () -> m_driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        m_susan = new SusanSubsystem(hardwareMap, "Susan");
        m_spinCommand = new spinSusan(m_susan, () -> m_driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER), () -> m_driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER));

        m_toggleButton = new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(m_grabCommand, m_releaseCommand);

        register(m_claw);
        register(m_lift);
        register(m_susan);

        m_lift.setDefaultCommand(m_heightCommand);
        m_susan.setDefaultCommand(m_spinCommand);
    }


}