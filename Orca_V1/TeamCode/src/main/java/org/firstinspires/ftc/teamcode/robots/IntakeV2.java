package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeV2 {
    public IntakeV2(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
    tilt = hardwareMap.get(ServoImplEx.class,"tilt");
    rotation = hardwareMap.get(DcMotorEx.class,"rotation");
    gate = hardwareMap.get(ServoImplEx.class,"gate");

    intake = hardwareMap.get(DcMotorEx.class,"intake");
    slides = hardwareMap.get(DcMotorEx.class,"hSlides");

    currentRotation = hardwareMap.get(AnalogInput.class, "currentRotation");
    currentTilt = hardwareMap.get(AnalogInput.class, "currentTilt");
}
