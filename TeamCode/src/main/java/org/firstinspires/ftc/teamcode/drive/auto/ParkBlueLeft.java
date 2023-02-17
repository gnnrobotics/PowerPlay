package org.firstinspires.ftc.teamcode.drive.auto;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.downLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.highLevel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.commands.Grab;
import org.firstinspires.ftc.teamcode.robot.commands.Release;
import org.firstinspires.ftc.teamcode.robot.commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.robot.commands.liftPIDCommand;
import org.firstinspires.ftc.teamcode.robot.commands.susanPIDCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SusanSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.auto.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.auto.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;


@Autonomous(name = "BLUE_LEFT_PARKING", group = "PARKING", preselectTeleOp = "FINAL_TELEOP")
public class ParkBlueLeft extends CommandOpMode {
// 2900 to 2750
    private MecanumSubsystem drive;

    private SelectCommand m_signalPark;
    private Pose2d startPose = new Pose2d(36.224, 61, Math.toRadians(-90.00));

    private ClawSubsystem m_claw;
    private Grab m_grabCommand;

    private LiftSubsystem m_lift;

    private SusanSubsystem m_susan;

    private AprilTagSubsystem m_aprilTag;
    public WaitUntilCommand DETECTOR_WAIT;
// 713 to 1230
    @Override
    public void initialize() {
        drive = new MecanumSubsystem(new SampleMecanumDrive(hardwareMap), true); // if i run into issues switch boolean

        m_lift = new LiftSubsystem(hardwareMap, "Lift");

        m_claw = new ClawSubsystem(hardwareMap);
        m_grabCommand = new Grab(m_claw);

        m_susan = new SusanSubsystem(hardwareMap, "susan");

        drive.setPoseEstimate(startPose);

        m_aprilTag = new AprilTagSubsystem(hardwareMap, "Webcam 1", 1280, 720, 0.4, 1552.74274588, 1552.74274588, 793.573231003, 202.006088244);
        DETECTOR_WAIT = new WaitUntilCommand(m_aprilTag::foundZone);
        m_aprilTag.init();

        /*

      .lineTo(new Vector2d(60.55, 61.67))
.lineTo(new Vector2d(60.92, 34.83))
.lineTo(new Vector2d(35.77, 35.20))
.splineTo(new Vector2d(36.14, 14.17), Math.toRadians(-87.32))
.lineTo(new Vector2d(57.12, 12.65))



         */

        TrajectorySequence terminalCone = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(60.17, 61.11))
                .build();
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(terminalCone.end())
                .lineTo(new Vector2d(60.92, 34.83))
                .build();


        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToStack.end())
                .lineTo(new Vector2d(12.86, 35.01))
                .build();
        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(goToStack.end())
                .lineTo(new Vector2d(35.01, 34.64))
                .splineTo(new Vector2d(35.01, 11.73), Math.toRadians(270.00))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(goToStack.end())
                .lineTo(new Vector2d(35.01, 34.64))
                .splineTo(new Vector2d(35.53, 10), Math.toRadians(-90))
                .lineTo(new Vector2d(59, 10))
                .build();

        m_signalPark = new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(AprilTagSubsystem.ParkingZone.LEFT, new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(drive, terminalCone),
                            new InstantCommand(m_claw::release),
                            new TrajectoryFollowerCommand(drive, goToStack),
                            new TrajectoryFollowerCommand(drive, parkLeft))
                );
                    put(AprilTagSubsystem.ParkingZone.CENTER, new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(drive, terminalCone),
                            new InstantCommand(m_claw::release),
                            new TrajectoryFollowerCommand(drive, goToStack),
                            new TrajectoryFollowerCommand(drive, parkCenter)));
                    put(AprilTagSubsystem.ParkingZone.RIGHT, new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(drive, terminalCone),
                            new InstantCommand(m_claw::release),
                            new TrajectoryFollowerCommand(drive, goToStack),
                            new TrajectoryFollowerCommand(drive, parkRight)));
                    put(AprilTagSubsystem.ParkingZone.NONE, new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(drive, terminalCone),
                            new InstantCommand(m_claw::release),
                            new TrajectoryFollowerCommand(drive, goToStack),
                            new TrajectoryFollowerCommand(drive, parkCenter)));
                }},
                // the selector
                m_aprilTag::getParkingZone
        );

        register(drive);
        register(m_lift);
        register(m_claw);
        register(m_aprilTag);

        schedule(new SequentialCommandGroup(
                m_grabCommand,
                new WaitUntilCommand(this::isStarted),
                new WaitUntilCommand(m_aprilTag::foundZone),
                m_signalPark
        ));


        if (isStopRequested()) return;
    }
    public void run() {
        super.run();

        telemetry.addData("park", m_aprilTag.getParkingZone());
        telemetry.update();
    }
}