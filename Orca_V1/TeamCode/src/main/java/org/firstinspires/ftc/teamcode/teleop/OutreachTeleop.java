package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.OrcaV2;
import org.firstinspires.ftc.teamcode.robots.OutreachChassis;

//@TeleOp

public class OutreachTeleop extends OpMode {
    OutreachChassis chassis;

    @Override
    public void init() {
        chassis = new OutreachChassis(hardwareMap);
        chassis.teleopInit();
    }
    @Override
    public void init_loop(){
    }
    @Override
    public void start(){
    }
    @Override
    public void loop() {
        double LSX, LSY, RSX;
        LSX = gamepad1.left_stick_x*gamepad2.right_trigger;
        LSY = gamepad1.left_stick_y*gamepad2.right_trigger;
        RSX = gamepad1.right_stick_x*gamepad2.right_trigger;

        if(gamepad2.right_trigger == 0){
            LSX = gamepad2.left_stick_x;
            LSY = gamepad2.left_stick_y;
            RSX = gamepad2.right_stick_x;
        }

        chassis.refresh(LSX,LSY,RSX);

    }
    @Override
    public void stop() {

    }
}