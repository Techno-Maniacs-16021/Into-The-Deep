package org.firstinspires.ftc.teamcode.auton;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.OrcaV1;

public abstract class AutonActions  extends LinearOpMode {
    public OrcaV1 initRobot(double x, double y, double heading){
        OrcaV1 orcaV1 = new OrcaV1(hardwareMap,new Pose2d(x,y,heading));
        //orcaV1.init();
        return orcaV1;
    }
    public Action depositSample(OrcaV1 orcav1,int runningMS){
        return telemetryPacket -> {
                orcav1.deposit().setSample();
                if(orcav1.deposit().slidesReachedTarget()){
                    orcav1.deposit().depositSample();
                    //sleep(250);
                    return true;
                }
                else {

                }
                if(true){

                }
            return false;
        };
    }
}
