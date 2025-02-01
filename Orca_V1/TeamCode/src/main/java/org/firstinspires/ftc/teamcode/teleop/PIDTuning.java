package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.OrcaV2;
/*
@TeleOp
@Config
public class PIDTuning extends OpMode {

    public static double p,i,d,f,target;
    OrcaV2 orca;
    @Override
    public void init() {
        p = 0; i = 0; d = 0; f = 0; target = 0;
        orca = new OrcaV2(hardwareMap,new Pose(0,0,0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        orca.intake().startIntaking();
    }
    @Override
    public void init_loop(){
        orca.intake().PIDTuning(p,i,d,f,target);
        orca.intake().refresh(0,false,false,false,false,true);
        telemetry.addData("Current Pos: ", orca.intake().getCurrentSlidePosition());
        telemetry.addData("Target Pos: ", target);
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {

    }
    @Override
    public void stop() {

    }

}*/
