package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

public class BasicRobot implements Robot{
    public DriveTrain driveTrain;
    @Override
    public void init(HardwareMap hardwareMap) {
        driveTrain = new DriveTrain(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        driveTrain.stop();
    }
}
