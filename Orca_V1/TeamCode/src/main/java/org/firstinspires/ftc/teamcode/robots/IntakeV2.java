package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeV2 {
    public IntakeV2(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
}
