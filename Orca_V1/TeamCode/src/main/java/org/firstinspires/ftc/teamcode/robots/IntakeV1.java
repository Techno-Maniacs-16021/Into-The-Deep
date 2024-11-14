package org.firstinspires.ftc.teamcode.robots;


import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IntakeV1 {
    //color sensor
    DigitalChannel colorPin0,colorPin1;
    //servos
    ServoImplEx tilt, rightRotation, leftRotation, gate;
    //motor
    DcMotorEx intake;

    double rotationPosition = 0, tiltPosition = 1, gatePosition = 0;
    int globalTime = 0;

    public IntakeV1(HardwareMap hardwareMap){
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //TODO: set configurations
        colorPin0 = hardwareMap.digitalChannel.get("crf0");
        colorPin1 = hardwareMap.digitalChannel.get("crf1");

        tilt = hardwareMap.get(ServoImplEx.class,"tilt");
        rightRotation = hardwareMap.get(ServoImplEx.class,"rRotation");
        leftRotation = hardwareMap.get(ServoImplEx.class,"lRotation");
        gate = hardwareMap.get(ServoImplEx.class,"gate");

        intake = hardwareMap.get(DcMotorEx.class,"intake");

        tilt.setPwmRange(new PwmControl.PwmRange(500,2500));
        rightRotation.setPwmRange(new PwmControl.PwmRange(500,2500));
        leftRotation.setPwmRange(new PwmControl.PwmRange(500,2500));
        gate.setPwmRange(new PwmControl.PwmRange(500,2500));

        tilt.setDirection(Servo.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void updateLoop (){
        tilt.setPosition(tiltPosition);
        rightRotation.setPosition(rotationPosition);
        leftRotation.setPosition(rotationPosition);
        gate.setPosition(gatePosition);

        colorEject();
    }
    public String sampleDetails(){
        return(
        colorPin0.getState()&&colorPin1.getState() ? "yellow"
        : !colorPin0.getState()&&colorPin1.getState() ? "red"
        : colorPin0.getState()&&!colorPin1.getState() ? "blue"
        : "none"
        );
    }
    public void neutralPosition(){
        tiltPosition = 0.5;
        rotationPosition = 0.5;
        //gatePosition = 0.5;
    }
    public void colorEject(){
        if(sampleDetails().equals("red")){
            gatePosition=0.8;
            intake.setPower(1);
        }
        else{
            gatePosition = 0;
            intake.setPower(0);
        }
    }
    public void rotateForward(int time){
        if((time-globalTime)>200){
            rotationPosition-=0.05;
            globalTime = time;
        }
    }
    public void rotateBackward(int time){
        if((time-globalTime)>200){
            rotationPosition+=0.05;
            globalTime = time;
        }
    }
    public void tiltUp(int time){
        if((time-globalTime)>200){
            tiltPosition+=0.05;
            globalTime = time;
        }
    }
    public void tiltDown(int time){
        if((time-globalTime)>200){
            tiltPosition-=0.05;
            globalTime = time;
        }
    }
    public void intake(double power){
        intake.setPower(power);
    }
}