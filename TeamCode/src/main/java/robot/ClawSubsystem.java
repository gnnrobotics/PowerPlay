package robot;

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

    public static final double leftOpen = 537.7;
    public static final double leftClosed = 537.7;

    public static final double rightOpen = 537.7;
    public static final double rightClosed = 537.7;

    public ClawSubsystem(final HardwareMap hMap, final String name) {
        leftServo = hMap.get(Servo.class, "leftServo");
        rightServo = hMap.get(Servo.class, "rightServo");

    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        leftServo.setPosition(0.76);
        rightServo.setPosition(0.76);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

}