package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DriveTrain {
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;

    public DriveTrain(HardwareMap hardwareMap){
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
    }

    public void drive(double drive, double turn){
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void stop(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
