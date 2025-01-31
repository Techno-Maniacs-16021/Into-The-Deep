package org.firstinspires.ftc.teamcode.auton.oldPaths;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.OrcaV1;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class AutonBase extends LinearOpMode {

    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;
    OpenCvWebcam webcam;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;


    public Action getArmToGround(OrcaV1 bot) {
        return telemetryPacket -> {
            sleep(150);
            return false;
        };
    }
}