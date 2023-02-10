package org.firstinspires.ftc.teamcode.drive.auto;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.downLevel;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.highLevel;

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

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
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


@Autonomous(name = "BLUE_LEFT_SCORING", group = "Scoring", preselectTeleOp = "FINAL_TELEOP")
public class BlueLeft11 extends CommandOpMode {

    private MecanumSubsystem drive;
    private TrajectoryFollowerCommand m_initialMovingCommand;
    private SequentialCommandGroup chainingTrajectories;
    private SelectCommand m_signalPark;
    private Pose2d startPose = new Pose2d(-36.00, 61.13, Math.toRadians(-90.00));

    private ClawSubsystem m_claw;;
    private Grab m_grabCommand;
    private Release m_releaseCommand;

    private LiftSubsystem m_lift;
    private liftPIDCommand m_levelCommand;
    private ParallelCommandGroup initialMoveAndLift;

    private SusanSubsystem m_susan;
    private susanPIDCommand m_susanCommand;

    private AprilTagSubsystem m_aprilTag;
    public WaitUntilCommand DETECTOR_WAIT;

    @Override
    public void initialize() {

        TrajectorySequence stackingCones = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-27.80, 4.02))
                .build();

        TrajectorySequence goToPickUpCone = drive.trajectorySequenceBuilder(stackingCones.end())
                .lineTo(new Vector2d(-65.30, 12.28))
                .build();


        TrajectorySequence goToDropOffCone = drive.trajectorySequenceBuilder(goToPickUpCone.end())
                .lineTo(new Vector2d(-27.80, 4.02))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToDropOffCone.end())
                .splineToSplineHeading(new Pose2d(-62.12, 23.94, Math.toRadians(90.00)), Math.toRadians(90.00))
                .build();
        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(goToDropOffCone.end())
                .lineTo(new Vector2d(-35.50, 24.28))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(goToDropOffCone.end())
                .lineTo(new Vector2d(-11.72, 24.45))
                .build();

        m_aprilTag = new AprilTagSubsystem(hardwareMap, "Webcam 1", 1280, 720, 0.4, 1552.74274588, 1552.74274588, 793.573231003, 202.006088244);
        DETECTOR_WAIT = new WaitUntilCommand(m_aprilTag::foundZone);
        m_aprilTag.init();

        drive = new MecanumSubsystem(new MecanumDrive(hardwareMap), true); // if i run into issues switch boolean
        m_initialMovingCommand = new TrajectoryFollowerCommand(drive, stackingCones);
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

        m_claw = new ClawSubsystem(hardwareMap);
        m_grabCommand = new Grab(m_claw);
        m_releaseCommand = new Release(m_claw);

        m_lift = new LiftSubsystem(hardwareMap, "Lift");
        m_levelCommand = new liftPIDCommand(m_lift, m_lift.getTarget());

        m_susan = new SusanSubsystem(hardwareMap, "Susan");
        m_susanCommand = new susanPIDCommand(m_susan, m_susan.getTarget());

        chainingTrajectories = new SequentialCommandGroup(
                m_grabCommand, // initially grab cone
                new ParallelCommandGroup( // offload cone
                    new TrajectoryFollowerCommand(drive, stackingCones),
                    new SequentialCommandGroup(
                            new WaitCommand(5000),
                            // new InstantCommand(() -> m_susan.setTarget(some value))
                            new InstantCommand(() -> m_lift.setTarget(highLevel)),
                            m_releaseCommand
                    )
                ),
                new ParallelCommandGroup( // go get another cone
                    new TrajectoryFollowerCommand(drive, goToPickUpCone),
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            // new InstantCommand(() -> m_susan.setTarget(some value))
                            new InstantCommand(() -> m_lift.setTarget(downLevel)) // lower lift
                    )
                ),
                m_grabCommand, // grab new cone
                new ParallelCommandGroup( // go drop off cone
                        new TrajectoryFollowerCommand(drive, goToDropOffCone),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                // new InstantCommand(() -> m_susan.setTarget(some value))
                                new InstantCommand(() -> m_lift.setTarget(highLevel))
                        ),
                        m_releaseCommand // release cone
                ),
                m_signalPark // park
        );


        register(drive);
        register(m_claw);
        register(m_lift);
        register(m_susan);
        register(m_aprilTag);

        m_lift.setDefaultCommand(m_levelCommand);
        m_susan.setDefaultCommand(m_susanCommand);

        schedule(chainingTrajectories);

        if (isStopRequested()) return;
    }
    public void run() {
        super.run();
        telemetry.addData("park", m_aprilTag.getParkingZone());
        telemetry.addData("liftTarget", m_lift.getTarget().getAsDouble());
        telemetry.addData("susanTarget", m_susan.getTarget().getAsDouble());
        telemetry.update();
    }
}