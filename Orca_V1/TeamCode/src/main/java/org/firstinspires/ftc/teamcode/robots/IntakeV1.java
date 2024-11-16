package org.firstinspires.ftc.teamcode.robots;


import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeV1 {
    //color sensor
    DigitalChannel colorPin0,colorPin1;
    //servos
    ServoImplEx tilt, rightRotation, leftRotation, gate;
    //motor
    DcMotorEx intake, slides;

    AnalogInput rotation;

    double rotationPosition = 0.7, tiltPosition = 0.9, gatePosition = 0;
    double ANGLED_ROTATION = 0.1, ANGLED_TILT = 0.2,
            VERTICAL_ROTATION = 0.25, VERTICAL_ROTATION_OFFSET = 0.35, VERTICAL_TILT = 0,
            RETRACT_ROTATION=0.7, RETRACT_TILT = 0.9,
            TRANSFER_ROTATION = 0.9, TRANSFER_TILT = 0.9;
    int globalTime = 0;
    String intakeCommand = "retract", intakeMode = "angled";
    String colorToEject = "red";
    double setSlidesPower = 0;

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
        slides = hardwareMap.get(DcMotorEx.class,"hSlides");

        rotation = hardwareMap.get(AnalogInput.class, "rotation");

        tilt.setPwmRange(new PwmControl.PwmRange(500,2500));
        rightRotation.setPwmRange(new PwmControl.PwmRange(500,2500));
        leftRotation.setPwmRange(new PwmControl.PwmRange(500,2500));
        gate.setPwmRange(new PwmControl.PwmRange(500,2500));

        tilt.setDirection(Servo.Direction.REVERSE);

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);

        //TODO: reset vars
        intakeCommand = "retract";
        intakeMode = "angled";

    }
    public void refresh (double slidePower,boolean cross, boolean circle, boolean triangle){
        tilt.setPosition(tiltPosition);
        rightRotation.setPosition(rotationPosition);
        leftRotation.setPosition(rotationPosition);
        gate.setPosition(gatePosition);
        //TODO: Make all the powers set using variables like above
        //colorEject();

        slideControlLoop(slidePower);
        intakeModuleControlLoop(cross,circle,triangle);

    }
    public String sampleDetails() {
        return (
                colorPin0.getState() && colorPin1.getState() ? "yellow"
                        : !colorPin0.getState() && colorPin1.getState() ? "red"
                        : colorPin0.getState() && !colorPin1.getState() ? "blue"
                        : "none"
        );
    }
       /*
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
    public void intakeSample(double power){
        intake.setPower(power);
    }
    */

    public void slideControlLoop(double slidePower){
        if(intakeCommand.equals("retract")){
            if(slides.getCurrent(CurrentUnit.AMPS)<9&&setSlidesPower!=-0.1){
                slides.setPower(-1);
                setSlidesPower = -1;
            }
            else{
                slides.setPower(-0.1);
                setSlidesPower = -0.1;
                if(!sampleDetails().equals("none")) {
                    intakeCommand = "transfer";
                }
            }
        }
        else if(intakeCommand.equals("transfer")){
            slides.setPower(-0.1);
        }
        else{
            slides.setPower(slidePower);
        }
    }
    public void intakeModuleControlLoop(boolean cross, boolean circle, boolean triangle){
        if(intakeCommand.equals("intake")){
            if(sampleDetails().equals(colorToEject)){
                gatePosition = 1;
                intake.setPower(0.5);
            }
            else if(intakeMode.equals("vertical")){
                gatePosition = 0;
                //intake vertically(90 degrees)
                //rotation set to a higher angle
                //moves down when intaking
                //returns to higher position when not intaking
                if(cross){
                    intake.setPower(1);
                    rotationPosition = VERTICAL_ROTATION;
                    tiltPosition = VERTICAL_TILT;
                }
                else if(triangle){
                    intake.setPower(-1);
                    rotationPosition = VERTICAL_ROTATION_OFFSET;
                    tiltPosition = VERTICAL_TILT;
                }
                else{
                    intake.setPower(0);
                    rotationPosition = VERTICAL_ROTATION_OFFSET;
                    tiltPosition = VERTICAL_TILT;
                }
            }
            else{
                gatePosition = 0;
                //intake flat on floor(45 degrees)
                //rotation set to lower angle
                //does not move up or down when intaking
                if(circle){
                    intake.setPower(1);
                }
                else if(triangle){
                    intake.setPower(-1);
                }
                else{
                    intake.setPower(0);
                }
                rotationPosition = ANGLED_ROTATION;
                tiltPosition = ANGLED_TILT;
            }
            //when button pressed, intake on. When button not pressed, intake off.
            //auto sample ejection
        }
        else if(intakeCommand.equals("retract")){
            rotationPosition = RETRACT_ROTATION;
            tiltPosition = RETRACT_TILT;
        }
        else if(intakeCommand.equals("transfer")){
            rotationPosition = TRANSFER_ROTATION;
            tiltPosition = TRANSFER_TILT;
            if(rotation.getVoltage()<0&&!sampleDetails().equals("none")){
                gatePosition = 1;
                intake.setPower(1);
            }
            else if(rotation.getVoltage()<0&&sampleDetails().equals("none")){
                gatePosition = 0;
                intake.setPower(0);
                intakeCommand = "retract";
            }
        }
    }
    public void retract(){
        intakeCommand = "retract";
    }

    public void transfer(){
        intakeCommand = "transfer";
    }
    public void startIntaking(){
        intakeCommand = "intake";
        intakeMode = "angled";
    }
    public void angledIntake(){
        intakeCommand = "intake";
        intakeMode = "angled";
    }
    public void verticalIntake(){
        intakeMode = "vertical";
    }
    public void setColorToEject(String color){
        colorToEject = color;
    }
    public String getColorToEject(){
        return colorToEject;
    }
    public String getIntakeCommand(){
        return intakeCommand;
    }

}