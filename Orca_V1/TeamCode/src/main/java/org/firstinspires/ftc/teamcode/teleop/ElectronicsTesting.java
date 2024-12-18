package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
@TeleOp
@Config
public class ElectronicsTesting extends OpMode {
    ServoImplEx servo0, servo1;
    DcMotorEx motor;
    DigitalChannel colorPin0,colorPin1;
    AnalogInput rotation, tilt;

    @Override
    public void init() {
        servo0 = hardwareMap.get(ServoImplEx.class,"servo0");
        servo1 = hardwareMap.get(ServoImplEx.class,"servo1");
        motor = hardwareMap.get(DcMotorEx.class,"motor");

        colorPin0 = hardwareMap.digitalChannel.get("crf0");
        colorPin1 = hardwareMap.digitalChannel.get("crf1");

        rotation = hardwareMap.get(AnalogInput.class, "rotation");
        tilt = hardwareMap.get(AnalogInput.class, "tilt");

        servo0.setPwmRange(new PwmControl.PwmRange(510,2490));
        servo1.setPwmRange(new PwmControl.PwmRange(510,2490));


    }
    @Override
    public void init_loop(){
        //servo0.setPosition(0);
        servo1.setPosition(1);
    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.right_stick_y);
        servo0.setPosition(Range.clip(gamepad1.left_stick_x+0.5,0,1));
        servo1.setPosition(Range.clip(gamepad1.left_stick_y+0.5,0,1));

        telemetry.addData("color: ",
                colorPin0.getState() && colorPin1.getState() ? "yellow"
                : !colorPin0.getState() && colorPin1.getState() ? "red"
                : colorPin0.getState() && !colorPin1.getState() ? "blue"
                : "none");
        telemetry.addData("rotation: ", rotation.getVoltage());
        telemetry.addData("tilt: ", tilt.getVoltage());
        telemetry.addData("pin 0: ", colorPin0.getState());
        telemetry.addData("pin 1: ", colorPin1.getState());
        telemetry.update();
    }
    @Override
    public void stop() {

    }

}
