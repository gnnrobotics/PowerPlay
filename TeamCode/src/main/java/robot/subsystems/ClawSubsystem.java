package robot.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class ClawSubsystem extends SubsystemBase {


    private final Servo leftServo;
    private final Servo rightServo;

    public static final double leftOpen = 0.4;
    public static final double leftClosed = 0;

    public static final double rightOpen = 0.4;
    public static final double rightClosed = 1;

    public ClawSubsystem(final HardwareMap hMap, final String name) {
        leftServo = hMap.get(Servo.class, "leftServo");
        rightServo = hMap.get(Servo.class, "rightServo");

    }

    /**
     * Grabs a cone.
     */
    public void grab() {
        leftServo.setPosition(leftClosed);
        rightServo.setPosition(rightClosed);
    }

    /**
     * Releases a cone.
     */
    public void release() {
        leftServo.setPosition(leftOpen);
        rightServo.setPosition(rightOpen);
    }
}