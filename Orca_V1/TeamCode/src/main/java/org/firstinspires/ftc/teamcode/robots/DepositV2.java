package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DepositV2 {
    public  DepositV2(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
}
