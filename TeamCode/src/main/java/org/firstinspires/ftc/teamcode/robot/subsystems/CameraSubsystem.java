package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.teamcode.robot.componentConstants.leftClosed;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.leftOpen;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.rightClosed;
import static org.firstinspires.ftc.teamcode.robot.componentConstants.rightOpen;

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

    public ClawSubsystem(final HardwareMap hMap) {
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