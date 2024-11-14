package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.localization.MecanumDrive;
public class OrcaV1 extends MecanumDrive{
    //IntakeV1 INTAKE;
    //DepositV1 DEPOSIT;
    IntakeV1 intake = new IntakeV1(hardwareMap);
    public OrcaV1(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        intake = new IntakeV1(hardwareMap);
    }

}