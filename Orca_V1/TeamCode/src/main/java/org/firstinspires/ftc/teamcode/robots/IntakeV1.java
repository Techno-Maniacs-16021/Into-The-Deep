package org.firstinspires.ftc.teamcode.robots;


import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeV1 {
    DigitalChannel colorPin0,colorPin1;
    public IntakeV1(HardwareMap hardwareMap){
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //TODO: set configurations
        colorPin0 = hardwareMap.digitalChannel.get("crf0");
        colorPin1 = hardwareMap.digitalChannel.get("crf1");
    }
    public static void updateLoop (){
        //	     P1 T	    P1 F
        //P0 T  yellow      red
        //P0 F   blue     nothing
    }
    public String sampleDetails(){
        boolean p1 = colorPin0.getState();
        return( 
        colorPin0.getState()&&colorPin1.getState() ? "Yellow"
        : colorPin0.getState()&&!colorPin1.getState() ? "Red" 
        : !colorPin0.getState()&&colorPin1.getState() ? "Blue" 
        : "None"
        );
    }
}