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
    AnalogInput a0, a1;

    @Override
    public void init() {
        servo0 = hardwareMap.get(ServoImplEx.class,"servo0");
        servo1 = hardwareMap.get(ServoImplEx.class,"servo1");
        motor = hardwareMap.get(DcMotorEx.class,"motor");


        a0 = hardwareMap.get(AnalogInput.class, "a0");
        a1 = hardwareMap.get(AnalogInput.class, "a1");

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
        motor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
        servo0.setPosition(Range.clip(gamepad1.right_stick_y+0.5,0,1));
        servo1.setPosition(Range.clip(gamepad1.left_stick_y+0.5,0,1));
        telemetry.addData("a0: ",a0.getVoltage());
        telemetry.addData("a1: ",a1.getVoltage());
        telemetry.update();
    }
    @Override
    public void stop() {

    }

}
