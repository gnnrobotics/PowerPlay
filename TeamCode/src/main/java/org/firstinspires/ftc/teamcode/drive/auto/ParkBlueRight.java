package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.robot.subsystems.auto.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.auto.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;


@Autonomous(name = "BLUE_RIGHT_PARKING", group = "PARKING", preselectTeleOp = "FINAL_TELEOP")
public class ParkBlueRight extends CommandOpMode {

    private MecanumSubsystem drive;
    private TrajectoryFollowerCommand m_initialMovingCommand;
    private SequentialCommandGroup chainingTrajectories;
    private SelectCommand m_signalPark;
    private Pose2d startPose = new Pose2d(36.00, 61.13, Math.toRadians(-90.00));



    private AprilTagSubsystem m_aprilTag;
    public WaitUntilCommand DETECTOR_WAIT;

    @Override
    public void initialize() {


        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(62.12, 23.94, Math.toRadians(-90.00)), Math.toRadians(-90.00))
                .build();
        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(35.50, 24.28))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(11.72, 24.5, Math.toRadians(-90.00)), Math.toRadians(-90.00))
                .build();

        m_aprilTag = new AprilTagSubsystem(hardwareMap, "Webcam 1", 1280, 720, 0.4, 1552.74274588, 1552.74274588, 793.573231003, 202.006088244);
        DETECTOR_WAIT = new WaitUntilCommand(m_aprilTag::foundZone);
        m_aprilTag.init();

        drive = new MecanumSubsystem(new MecanumDrive(hardwareMap), true); // if i run into issues switch boolean
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
        register(m_aprilTag);


        schedule(m_signalPark);

        if (isStopRequested()) return;
    }
    public void run() {
        super.run();
        telemetry.addData("park", m_aprilTag.getParkingZone());
        telemetry.update();
    }
}