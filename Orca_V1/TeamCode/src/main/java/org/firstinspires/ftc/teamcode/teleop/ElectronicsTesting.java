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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
@TeleOp
@Config
public class ElectronicsTesting extends OpMode {
    ServoImplEx tilt, lDiff, rDiff, intRotation, depLinkage, depRotation, gate, claw;
    DcMotorEx leftBack, rightBack, leftFront, rightFront, leftVert, rightVert, horizontal, intake, m0, m1, m2, m3;
    DigitalChannel crf0,crf1,crf2,crf3,crf4,crf5,ccp6,ccp7,ecp0,ecp1,ecp2,ecp3,ecp4,ecp5,ecp6,ecp7;
    AnalogInput cTilt, cIntRotation, cDepRotation, cDepLinkage, cRightDiff, cLeftDiff, e2, e3;

    @Override
    public void init() {
        //claw = hardwareMap.get(ServoImplEx.class,"claw");
        //claw.setDirection(Servo.Direction.REVERSE);

        tilt = hardwareMap.get(ServoImplEx.class,"tilt");
        lDiff = hardwareMap.get(ServoImplEx.class,"ld");
        rDiff = hardwareMap.get(ServoImplEx.class,"rd");
        intRotation = hardwareMap.get(ServoImplEx.class,"introt");
        depLinkage = hardwareMap.get(ServoImplEx.class,"deplink");
        depRotation = hardwareMap.get(ServoImplEx.class,"deprot");
        gate = hardwareMap.get(ServoImplEx.class,"gate"); //no analog
        claw = hardwareMap.get(ServoImplEx.class,"claw");

        leftBack = hardwareMap.get(DcMotorEx.class,"lb");
        leftFront = hardwareMap.get(DcMotorEx.class,"lf");
        rightBack = hardwareMap.get(DcMotorEx.class,"rb");
        rightFront = hardwareMap.get(DcMotorEx.class,"rf");

        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftVert = hardwareMap.get(DcMotorEx.class,"vLSlides");
        rightVert = hardwareMap.get(DcMotorEx.class,"vRSlides");
        horizontal = hardwareMap.get(DcMotorEx.class,"intake");
        intake = hardwareMap.get(DcMotorEx.class,"hSlides");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        crf0 = hardwareMap.digitalChannel.get("crf0");
        crf1 = hardwareMap.digitalChannel.get("crf1");
        crf2 = hardwareMap.digitalChannel.get("crf2");
        crf3 = hardwareMap.digitalChannel.get("crf3");
        crf4 = hardwareMap.digitalChannel.get("crf4");
        crf5 = hardwareMap.digitalChannel.get("crf5");
        /*ccp6 = hardwareMap.digitalChannel.get("crf6");
        ccp7 = hardwareMap.digitalChannel.get("crf7");
        ecp0 = hardwareMap.digitalChannel.get("erf0");
        ecp1 = hardwareMap.digitalChannel.get("erf1");
        ecp2 = hardwareMap.digitalChannel.get("erf2");
        ecp3 = hardwareMap.digitalChannel.get("erf3");
        ecp4 = hardwareMap.digitalChannel.get("erf4");
        ecp5 = hardwareMap.digitalChannel.get("erf5");
        ecp6 = hardwareMap.digitalChannel.get("erf6");
        ecp7 = hardwareMap.digitalChannel.get("erf7");*/

        cTilt = hardwareMap.get(AnalogInput.class, "ctilt");
        cIntRotation = hardwareMap.get(AnalogInput.class, "cintrot");
        cDepRotation = hardwareMap.get(AnalogInput.class, "cdeprot");
        cDepLinkage = hardwareMap.get(AnalogInput.class, "cdeplink");
        cRightDiff = hardwareMap.get(AnalogInput.class, "crd");
        cLeftDiff = hardwareMap.get(AnalogInput.class, "cld");



// set the clock speed on this I2C bus to 400kHz:



        depLinkage.setPwmRange(new PwmControl.PwmRange(510,2490));
        depRotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        claw.setPwmRange(new PwmControl.PwmRange(510,2490));
        intRotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        gate.setPwmRange(new PwmControl.PwmRange(510,2490));


    }
    @Override
    public void init_loop(){
        //servo0.setPosition(0);
        //servo1.setPosition(1);
        intRotation.setPosition(0.85);
    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {
        //horizontal.setPower(gamepad1.left_trigger);
        //intake.setPower(gamepad1.right_trigger);
        //intRotation.setPosition(0.5);
        //gate.setPosition(1);
        rDiff.setPosition(Range.clip(gamepad1.right_stick_y+0.5,0,1));
        lDiff.setPosition(Range.clip(gamepad1.left_stick_y+0.5,0,1));
        claw.setPosition(Range.clip(gamepad1.right_stick_x+0.5,0,1));
        depRotation.setPosition(Range.clip(gamepad1.left_stick_x+0.5,0,1));
        depLinkage.setPosition(Range.clip(gamepad2.left_stick_x,0,1));
        gate.setPosition(Range.clip(gamepad2.right_stick_x+0.5,0,1));
        tilt.setPosition(Range.clip(gamepad2.right_stick_y+0.5,0,1));
        intRotation.setPosition(Range.clip(gamepad2.left_stick_y+0.5,0,1));

        leftFront.setPower(gamepad1.left_trigger);
        rightFront.setPower(gamepad1.right_trigger);
        leftFront.setPower(-gamepad2.left_trigger);
        rightFront.setPower(-gamepad2.right_trigger);



        telemetry.addData("tilt: ",cTilt.getVoltage());
        telemetry.addData("deplink: ",cDepLinkage.getVoltage());
        telemetry.addData("introt: ",cIntRotation.getVoltage());
        telemetry.addData("deprot: ",cDepRotation.getVoltage());
        telemetry.addData("ld: ",cLeftDiff.getVoltage());
        telemetry.addData("rd: ",cRightDiff.getVoltage());
        //telemetry.addData("e2: ",e2.getVoltage());
        //telemetry.addData("e3: ",e3.getVoltage());
        telemetry.addData("crf0:", crf0.getState());
        telemetry.addData("crf1:", crf1.getState());
        telemetry.addData("crf2:", crf2.getState());
        telemetry.addData("crf3:", crf3.getState());
        telemetry.addData("crf4:", crf4.getState());
        telemetry.addData("crf5:", crf5.getState());
        telemetry.update();

    }
    @Override
    public void stop() {

    }

}
