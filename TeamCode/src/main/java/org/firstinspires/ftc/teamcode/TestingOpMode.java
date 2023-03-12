package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.commands.FineDriveCommand;
import org.firstinspires.ftc.teamcode.robot.commands.liftPIDCommand;
import org.firstinspires.ftc.teamcode.robot.commands.setSpecificHeight;
import org.firstinspires.ftc.teamcode.robot.commands.spinSusan;
import org.firstinspires.ftc.teamcode.robot.commands.susanPIDCommand;
import org.firstinspires.ftc.teamcode.robot.componentConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.commands.Grab;
import org.firstinspires.ftc.teamcode.robot.commands.Release;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;

@TeleOp(name = "NO_PID_TELEOP") // ALWAYS RUN AUTO AT 12.53 or above

public class TestingOpMode extends CommandOpMode { // remember guy on discord who gave u his liftsubsystem, also 1 and 3 were swapped

    private GamepadEx m_driverOp;
    private GamepadEx m_coOp;

    private DriveSubsystem m_drive;
    private DriveCommand m_driveCommand;
    Button m_fineDriveButton;

    private ClawSubsystem m_claw;;
    private Grab m_grabCommand;
    private Release m_releaseCommand;

    private LiftSubsystem m_lift;
    private SequentialCommandGroup m_manualANDPid;
    private setSpecificHeight m_heightCommand;
    private liftPIDCommand m_levelCommand;

    private SusanSubsystem m_susan;
    private spinSusan m_spinCommand;
    private FineDriveCommand m_fineDriveCommand;
    Button m_toggleButton;

    @Override
    public void initialize() {
// organize by subsystems then commands
        m_driverOp = new GamepadEx(gamepad1);
        m_coOp = new GamepadEx(gamepad2);

        m_drive = new DriveSubsystem(hardwareMap);
        m_driveCommand = new DriveCommand(m_drive, () -> m_driverOp.getLeftX(), () -> m_driverOp.getLeftY(), () -> m_driverOp.getRightX());
        m_fineDriveCommand = new FineDriveCommand(m_drive, () -> m_driverOp.getLeftX(), () -> m_driverOp.getLeftY(), () -> m_driverOp.getRightX());

        m_claw = new ClawSubsystem(hardwareMap);
        m_grabCommand = new Grab(m_claw);
        m_releaseCommand = new Release(m_claw);

        m_lift = new LiftSubsystem(hardwareMap, "Lift", false);
        m_heightCommand = new setSpecificHeight(m_lift, () -> m_driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), () -> m_driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        m_coOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            m_lift.setLevel(componentConstants.Level.DOWN);
        }));
        m_coOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            m_lift.setLevel(componentConstants.Level.GROUND_J);
        }));
        m_coOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            m_lift.setLevel(componentConstants.Level.LOW);
        }));
        m_coOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> {
            m_lift.setLevel(componentConstants.Level.MEDIUM);
        }));
        m_coOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            m_lift.setLevel(componentConstants.Level.HIGH);
        }));


        m_susan = new SusanSubsystem(hardwareMap, "Susan");
        m_spinCommand = new spinSusan(
                m_susan,
                () -> m_driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER), () -> m_driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER),
                () -> m_coOp.getButton(GamepadKeys.Button.LEFT_BUMPER), () -> m_coOp.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        m_fineDriveButton = new GamepadButton(m_coOp, GamepadKeys.Button.B).whenHeld(m_fineDriveCommand);
        m_toggleButton = new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(m_grabCommand, m_releaseCommand);

        register(m_drive);
        register(m_claw);
        register(m_lift);
        register(m_susan);

        m_drive.setDefaultCommand(m_driveCommand);
        m_lift.setDefaultCommand(m_heightCommand);
        m_susan.setDefaultCommand(m_spinCommand);
    }
    public void run() {
        super.run();
        //vroom vroom
        telemetry.addData("fL", m_drive.getFL());
        telemetry.addData("fR", m_drive.getFR());
        telemetry.addData("bL", m_drive.getBL());
        telemetry.addData("bR",m_drive.getBR());

        telemetry.addData("fLE", m_drive.getFLEncoder());
        telemetry.addData("fRE", m_drive.getFREncoder());
        telemetry.addData("bLE", m_drive.getBLEncoder());
        telemetry.addData("bRE",m_drive.getBREncoder());

        telemetry.addData("pos", m_lift.getEncoder());
        telemetry.addData("power", m_lift.getPower());
        telemetry.addData("level", componentConstants.currentLevel);
        telemetry.update();
    }
}