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

import java.util.ArrayList;

public class IntakeV1 {
    //color sensor
    DigitalChannel colorPin0,colorPin1;
    //servos
    ServoImplEx tilt, rotation, gate;
    //motor
    DcMotorEx intake, slides;

    AnalogInput currentRotation, currentTilt;

    double rotationPosition = 0.6, tiltPosition = 0.6, gatePosition = 0;
    double ANGLED_ROTATION = 0.08, ANGLED_TILT = 0.25,
            VERTICAL_ROTATION = .11, VERTICAL_ROTATION_OFFSET = 0.25, VERTICAL_TILT = 0.15,
            RETRACT_ROTATION = 0.25, RETRACT_TILT = .6,
            TRANSFER_ROTATION = 1, INTER_TRANSFER_ROTATION = 0.7, TRANSFER_TILT = 0.95,
            STANDBY_ROTATION = 0.6, STANDBY_TILT = 0.6;
    int globalTime = 0;
    String intakeCommand = "standby", intakeMode = "angled";
    String colorToEject = "red";
    double slidesPower = 0, intakePower = 0;
    int nSensorSamples = 50;
    ArrayList<String> colorSensorInputs = new ArrayList<String>();

    public IntakeV1(HardwareMap hardwareMap){
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //TODO: set configurations
        colorPin0 = hardwareMap.digitalChannel.get("crf0");
        colorPin1 = hardwareMap.digitalChannel.get("crf1");

        tilt = hardwareMap.get(ServoImplEx.class,"tilt");
        rotation = hardwareMap.get(ServoImplEx.class,"rotation");
        gate = hardwareMap.get(ServoImplEx.class,"gate");

        intake = hardwareMap.get(DcMotorEx.class,"intake");
        slides = hardwareMap.get(DcMotorEx.class,"hSlides");

        currentRotation = hardwareMap.get(AnalogInput.class, "currentRotation");
        currentTilt = hardwareMap.get(AnalogInput.class, "currentTilt");

        tilt.setPwmRange(new PwmControl.PwmRange(510,2490));
        rotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        gate.setPwmRange(new PwmControl.PwmRange(510,2490));

        tilt.setDirection(Servo.Direction.REVERSE);
        //gate.setDirection(Servo.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //slides.setDirection(DcMotorSimple.Direction.REVERSE);

        //TODO: reset vars
        intakeCommand = "retract";
        intakeMode = "angled";

        for(int i = 0; i < nSensorSamples; i++){
            colorSensorInputs.add("none");
        }
    }
    public void refresh (double rightTrigger,boolean cross, boolean circle, boolean triangle){

        tilt.setPosition(tiltPosition);
        rotation.setPosition(rotationPosition);
        gate.setPosition(gatePosition);
        slides.setPower(slidesPower);
        if(colorEject()==1){
            intake.setPower(1);
        }
        else if(colorEject()==-1){
            intake.setPower(-1);
        }
        else {
            intake.setPower(intakePower);
        }

        slideControlLoop(rightTrigger);
        intakeModuleControlLoop(cross,circle,triangle);
        updateSampleDetails();

    }
    public String readSampleDetails() {
        int blue = 0, red = 0, yellow = 0, none = 0;
        for(String color:colorSensorInputs){
            if(color.equals("blue"))
                blue++;
            else if(color.equals("red"))
                red++;
            else if(color.equals("yellow"))
                yellow++;
            else
                none++;
        }
        return (
                blue>red && blue>yellow && blue>none ? "blue"
                        : red>blue && red>yellow && red>none ? "red"
                        : yellow>red && yellow>blue && yellow>none ? "yellow"
                        : "none"
        );
    }
    public void updateSampleDetails(){
        colorSensorInputs.add(
                colorPin0.getState() && colorPin1.getState() ? "yellow"
                        : !colorPin0.getState() && colorPin1.getState() ? "red"
                        : colorPin0.getState() && !colorPin1.getState() ? "blue"
                        : "none");
        colorSensorInputs.remove(0);
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
    public double colorEject(){
        return 0.0;
    }
    public void slideControlLoop(double slidesPower){
        if(intakeCommand.equals("retract")||intakeCommand.equals("standby")){
            if(slides.getCurrent(CurrentUnit.AMPS)<7&&this.slidesPower!=-0.25){
                this.slidesPower = -1;
            }
            else{
                this.slidesPower = -0.25;
                if(!readSampleDetails().equals("none")) {
                    intakeCommand = "transfer";
                    intakePower = -0.5;
                }
                else {
                    intakeCommand = "standby";
                    intakePower = 0;
                }
            }
        }
        else if(intakeCommand.equals("transfer")){
            this.slidesPower = -0.25;
        }
        else{
            this.slidesPower = slidesPower;
        }
    }
    public void intakeModuleControlLoop(boolean cross, boolean circle, boolean triangle){
        if(intakeCommand.equals("intake")){
//            if(sampleDetails().equals(colorToEject)){
//                //gatePosition = 1;
//                //intake.setPower(0.3);
//            }
//            else
                if(intakeMode.equals("vertical")){
                gatePosition = 0;
                //intake vertically(90 degrees)
                //rotation set to a higher angle
                //moves down when intaking
                //returns to higher position when not intaking
                if(cross){
                    intakePower = 1;
                    rotationPosition = VERTICAL_ROTATION;
                    tiltPosition = VERTICAL_TILT;
                }
                else if(triangle){
                    intakePower = -1;
                    rotationPosition = VERTICAL_ROTATION_OFFSET;
                    tiltPosition = VERTICAL_TILT;
                }
                else{
                    intakePower = 0;
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
                    intakePower = 1;
                }
                else if(triangle){
                    intakePower = -1;
                }
                else{
                    intakePower = 0;
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
            intakePower = 0.5;
        }
        else if(intakeCommand.equals("standby")){
            rotationPosition = STANDBY_ROTATION;
            tiltPosition = STANDBY_TILT;
            intakePower = 0;
        }
        else if(intakeCommand.equals("transfer")){
            if(currentTilt.getVoltage()>1.25&&rotationPosition != TRANSFER_ROTATION){
                rotationPosition = INTER_TRANSFER_ROTATION;
            }
            else{
                rotationPosition = TRANSFER_ROTATION;
            }
            tiltPosition = TRANSFER_TILT;
            if(currentRotation.getVoltage()<1.2&&!readSampleDetails().equals("none")){
                gatePosition = 1;
                intakePower = 1;
            }
            else if(readSampleDetails().equals("none")){
                gatePosition = 0;
                intakePower = 0;
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
        intakeMode = "vertical";
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
    public AnalogInput currentRotation(){
        return currentRotation;
    }
    public AnalogInput currentTilt(){
        return currentTilt;
    }

}