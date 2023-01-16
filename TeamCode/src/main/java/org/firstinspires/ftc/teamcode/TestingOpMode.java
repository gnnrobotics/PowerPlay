package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.commands.FineDriveCommand;
import org.firstinspires.ftc.teamcode.robot.commands.setSpecificHeight;
import org.firstinspires.ftc.teamcode.robot.commands.spinSusan;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.commands.Grab;
import org.firstinspires.ftc.teamcode.robot.commands.Release;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

@TeleOp(name = "TestingClaw")

public class TestingOpMode extends CommandOpMode {

    private GamepadEx m_driverOp;

    private DriveSubsystem m_drive;
    private DriveCommand m_driveCommand;
    Button m_fineDriveButton;

    private ClawSubsystem m_claw;;
    private Grab m_grabCommand;
    private Release m_releaseCommand;

    private LiftSubsystem m_lift;
    private setSpecificHeight m_heightCommand;

    private SusanSubsystem m_susan;
    private spinSusan m_spinCommand;
    private FineDriveCommand m_fineDriveCommand;
    Button m_toggleButton;

    @Override
    public void initialize() {
// organize by subsystems then commands
        m_driverOp = new GamepadEx(gamepad1);

        m_drive = new DriveSubsystem(hardwareMap);
        m_driveCommand = new DriveCommand(m_drive, () -> m_driverOp.getLeftX(), () -> m_driverOp.getLeftY(), () -> m_driverOp.getRightX());
        m_fineDriveCommand = new FineDriveCommand(m_drive, () -> m_driverOp.getLeftX(), () -> m_driverOp.getLeftY(), () -> m_driverOp.getRightX());

        m_claw = new ClawSubsystem(hardwareMap);
        m_grabCommand = new Grab(m_claw);
        m_releaseCommand = new Release(m_claw);

        m_lift = new LiftSubsystem(hardwareMap, "Lift");
        m_heightCommand = new setSpecificHeight(m_lift, () -> m_driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), () -> m_driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        m_susan = new SusanSubsystem(hardwareMap, "Susan");
        m_spinCommand = new spinSusan(m_susan, () -> m_driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER), () -> m_driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER));

        m_fineDriveButton = new GamepadButton(m_driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON).whenHeld(m_fineDriveCommand);
        m_toggleButton = new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(m_grabCommand, m_releaseCommand);

        register(m_drive);
        register(m_claw);
        register(m_lift);
        register(m_susan);

        m_drive.setDefaultCommand(m_driveCommand);
        m_lift.setDefaultCommand(m_heightCommand);
        m_susan.setDefaultCommand(m_spinCommand);
    }


}