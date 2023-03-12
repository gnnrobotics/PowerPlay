package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.commands.Grab;
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


@Autonomous(name = "BLUE_LEFT_SCORING", group = "SCORING", preselectTeleOp = "FINAL_TELEOP")
public class BlueLeft11 extends CommandOpMode {

    private MecanumSubsystem drive;
    private SelectCommand m_signalPark;
    private Pose2d startPose = new Pose2d(36.224, 63, Math.toRadians(-90.00));

    private ClawSubsystem m_claw;
    private Grab m_grabCommand;

    private LiftSubsystem m_lift;
    private liftPIDCommand liftPIDCommand;

    private SusanSubsystem m_susan;
    private susanPIDCommand susanPIDCommand;

    private AprilTagSubsystem m_aprilTag;
    public WaitUntilCommand DETECTOR_WAIT;

    @Override
    public void initialize() {
        drive = new MecanumSubsystem(new SampleMecanumDrive(hardwareMap), true); // if i run into issues switch boolean

        m_claw = new ClawSubsystem(hardwareMap);
        m_grabCommand = new Grab(m_claw);

        m_lift = new LiftSubsystem(hardwareMap, "lift", true);

        m_susan = new SusanSubsystem(hardwareMap, "susan");

        drive.setPoseEstimate(startPose);

        m_aprilTag = new AprilTagSubsystem(hardwareMap, "Webcam 1", 1280, 720, 0.4, 1552.74274588, 1552.74274588, 793.573231003, 202.006088244);
        DETECTOR_WAIT = new WaitUntilCommand(m_aprilTag::foundZone);
        m_aprilTag.init();

        TrajectorySequence goToConeInitial = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(35.01, 14.93), Math.toRadians(90.00))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11.81, 61.50))
                .lineTo(new Vector2d(13.50, 24.94))
                .build();
        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36.19, 24.56))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(58, 63))
                .lineTo(new Vector2d(58, 36))
                .build();

       liftPIDCommand = new liftPIDCommand(m_lift);
       susanPIDCommand = new susanPIDCommand(m_susan);

        m_signalPark = new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(AprilTagSubsystem.ParkingZone.LEFT, new TrajectoryFollowerCommand(drive, parkLeft));
                    put(AprilTagSubsystem.ParkingZone.CENTER, new TrajectoryFollowerCommand(drive, parkCenter));
                    put(AprilTagSubsystem.ParkingZone.RIGHT,  new TrajectoryFollowerCommand(drive, parkRight));
                    put(AprilTagSubsystem.ParkingZone.NONE, new TrajectoryFollowerCommand(drive, parkCenter));
                }},
                // the selector
                m_aprilTag::getParkingZone
        );

        register(drive);
        register(m_claw);
        register(m_aprilTag);

        m_lift.setDefaultCommand(liftPIDCommand);
        m_susan.setDefaultCommand(susanPIDCommand);
// set a system where you can easily set the target value, maybe use the existing pid commands
        schedule(
        );


       /* schedule( this is for movement, but we're testing and that's annoying
                new SequentialCommandGroup(
                        m_grabCommand,
                        new WaitUntilCommand(this::isStarted).withTimeout(2000),
                        new WaitUntilCommand(m_aprilTag::foundZone),
                        new TrajectoryFollowerCommand(drive, goToConeInitial)
                )); */


        if (isStopRequested()) return;
    }
    public void run() {
        super.run();

        telemetry.addData("park", m_aprilTag.getParkingZone());
        telemetry.addData("liftUseState", m_lift.getUseState());
        telemetry.addData("target", m_lift.getTarget().getAsDouble());
        telemetry.update();
    }
}