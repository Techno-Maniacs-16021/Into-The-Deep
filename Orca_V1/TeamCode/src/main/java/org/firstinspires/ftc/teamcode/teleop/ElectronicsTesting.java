package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
@TeleOp
@Config
public class ElectronicsTesting extends OpMode {
    ServoImplEx intake, lDiff, rDiff, intRotation, depLinkage, depRotation, gate, claw;
    DcMotorEx leftBack, rightBack, leftFront, rightFront;
    DigitalChannel colorPin0,colorPin1,colorPin2,colorPin3,colorPin4,colorPin5;
    AnalogInput c0, c1, c2, c3, e0, e1, e2, e3;

    @Override
    public void init() {
        //intake = hardwareMap.get(ServoImplEx.class,"int");
        //lDiff = hardwareMap.get(ServoImplEx.class,"ld");
        //rDiff = hardwareMap.get(ServoImplEx.class,"rd");
        //intRotation = hardwareMap.get(ServoImplEx.class,"introt");
        //depLinkage = hardwareMap.get(ServoImplEx.class,"deplink");
        //depRotation = hardwareMap.get(ServoImplEx.class,"deprot");
        //gate = hardwareMap.get(ServoImplEx.class,"gate"); //no analog
        //claw = hardwareMap.get(ServoImplEx.class,"claw");

        leftBack = hardwareMap.get(DcMotorEx.class,"lb");
        leftFront = hardwareMap.get(DcMotorEx.class,"lf");
        rightBack = hardwareMap.get(DcMotorEx.class,"rb");
        rightFront = hardwareMap.get(DcMotorEx.class,"rf");

        //leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        colorPin0 = hardwareMap.digitalChannel.get("crf0");
        colorPin1 = hardwareMap.digitalChannel.get("crf1");
        colorPin2 = hardwareMap.digitalChannel.get("crf2");
        colorPin3 = hardwareMap.digitalChannel.get("crf3");
        colorPin4 = hardwareMap.digitalChannel.get("crf4");
        colorPin5 = hardwareMap.digitalChannel.get("crf5");



// set the clock speed on this I2C bus to 400kHz:



        //depLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        //depRotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        //claw.setPwmRange(new PwmControl.PwmRange(510,2490));
        //intRotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        //gate.setPwmRange(new PwmControl.PwmRange(510,2490));


    }
    @Override
    public void init_loop(){
        //servo0.setPosition(0);
        //servo1.setPosition(1);
    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {
        leftFront.setPower(gamepad1.left_trigger);
        rightFront.setPower(gamepad1.right_trigger);
        rightBack.setPower(gamepad2.right_trigger);
        leftBack.setPower(gamepad2.left_trigger);
        //intRotation.setPosition(0.5);
        //gate.setPosition(1);
        //intRotation.setPosition(Range.clip(gamepad1.right_stick_y+0.5,0,1));
        //intake.setPosition(Range.clip(gamepad1.left_stick_y+0.5,0,1));
        //claw.setPosition(Range.clip(gamepad1.right_stick_x+0.5,0,1));
        //intRotation.setPosition(Range.clip(gamepad1.left_stick_x+0.5,0,1));


        /*telemetry.addData("c0: ",c0.getVoltage());
        telemetry.addData("c1: ",c1.getVoltage());
        //telemetry.addData("c2: ",c2.getVoltage());
        telemetry.addData("c3: ",c3.getVoltage());
        //telemetry.addData("e0: ",e0.getVoltage());
        //telemetry.addData("e1: ",e1.getVoltage());
        //telemetry.addData("e2: ",e2.getVoltage());
        telemetry.addData("e3: ",e3.getVoltage());
        telemetry.addData("crf0:", colorPin0.getState());
        telemetry.addData("crf1:", colorPin1.getState());
        telemetry.addData("crf2:", colorPin2.getState());
        telemetry.addData("crf3:", colorPin3.getState());
        telemetry.addData("crf4:", colorPin4.getState());
        telemetry.addData("crf5:", colorPin5.getState());
        telemetry.update();*/
    }
    @Override
    public void stop() {

    }

}
