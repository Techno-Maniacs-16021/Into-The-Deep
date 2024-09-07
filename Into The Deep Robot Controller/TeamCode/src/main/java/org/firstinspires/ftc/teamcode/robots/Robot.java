package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Robot {
    void init(HardwareMap hardwareMap);
    void start();
    void stop();
}
