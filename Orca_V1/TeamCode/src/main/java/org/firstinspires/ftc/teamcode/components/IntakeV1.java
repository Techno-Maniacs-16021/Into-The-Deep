package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeV1 {
    //color sensor
    DigitalChannel colorPin0,colorPin1;
    //servos
    ServoImplEx tilt, rightRotation, leftRotation, gate;
    //motor
    DcMotorEx intake, slides;

    AnalogInput rotation;

    double rotationPosition = 0.7, tiltPosition = 1, gatePosition = 0;
    double ANGLED_ROTATION = 0.15, ANGLED_TILT = 0.25,
            VERTICAL_ROTATION = 0.16, VERTICAL_ROTATION_OFFSET = 0.26, VERTICAL_TILT = 0.15,
            RETRACT_ROTATION=0.7, RETRACT_TILT = 1,
            TRANSFER_ROTATION = 1, TRANSFER_TILT = 1;
    int globalTime = 0;
    IntakeCommand intakeCommand;
    IntakeMode intakeMode;
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

        slides.setDirection(DcMotorSimple.Direction.REVERSE);

        //TODO: reset vars
        intakeCommand = IntakeCommand.RETRACT;
        intakeMode = IntakeMode.ANGLED;

    }
    public void refresh (double slidePower,boolean cross, boolean circle, boolean triangle){

        switch (intakeCommand) {
            case RETRACT:
                handleRetract();
                break;

            case TRANSFER:
                handleTransfer();
                break;

            case INTAKE:
                handleIntake(cross, circle, triangle);
                break;
        }


        //TODO: Make all the powers set using variables like above
        //colorEject();

        slideControlLoop(slidePower);
        tilt.setPosition(tiltPosition);
        rightRotation.setPosition(rotationPosition);
        leftRotation.setPosition(rotationPosition);
        gate.setPosition(gatePosition);
    }

    private void handleRetract() {
        setPositions(RETRACT_ROTATION, RETRACT_TILT, 0);
        slides.setPower(-0.1);

        if (!sampleDetails().equals("none")) {
            intakeCommand = IntakeCommand.TRANSFER;
        }
    }

    private void handleTransfer() {
        setPositions(TRANSFER_ROTATION, TRANSFER_TILT, gatePosition);

        if (rotation.getVoltage() < 1.05) {
            if (!sampleDetails().equals("none")) {
                gatePosition = 1;
                intake.setPower(1);
            } else {
                gatePosition = 0;
                intake.setPower(0);
                intakeCommand = IntakeCommand.RETRACT;
            }
        }
    }

    private void handleIntake(boolean cross, boolean circle, boolean triangle) {
        gatePosition = 0;

        if (intakeMode == IntakeMode.VERTICAL) {
            if (cross) {
                intake.setPower(1);
                setPositions(VERTICAL_ROTATION, VERTICAL_TILT, 0);
            } else if (triangle) {
                intake.setPower(-1);
                setPositions(VERTICAL_ROTATION_OFFSET, VERTICAL_TILT, 0);
            } else {
                intake.setPower(0);
                setPositions(VERTICAL_ROTATION_OFFSET, VERTICAL_TILT, 0);
            }
        } else { // ANGLED
            if (circle) {
                intake.setPower(1);
            } else if (triangle) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
            setPositions(ANGLED_ROTATION, ANGLED_TILT, 0);
        }
    }


    public String sampleDetails() {
        return (
                colorPin0.getState() && colorPin1.getState() ? "blue"
                        : !colorPin0.getState() && colorPin1.getState() ? "blue"
                        : colorPin0.getState() && !colorPin1.getState() ? "blue"
                        : "none"
        );
    }

    public void slideControlLoop(double slidePower){
        if(intakeCommand == IntakeCommand.RETRACT){
            if(slides.getCurrent(CurrentUnit.AMPS)<7&&setSlidesPower!=-0.1){
                slides.setPower(-1);
                setSlidesPower = -1;
            }
            else{
                slides.setPower(-0.1);
                setSlidesPower = -0.1;
                if(!sampleDetails().equals("none")) {
                    intakeCommand = IntakeCommand.TRANSFER;
                }
            }
        }
        else if(intakeCommand == IntakeCommand.TRANSFER){
            slides.setPower(-0.1);
            setSlidesPower = 0;
        }
        else{
            slides.setPower(slidePower);
            setSlidesPower = 0;
        }
    }

    public void retract(){
        intakeCommand = IntakeCommand.RETRACT;
    }

    public void transfer(){
        intakeCommand = IntakeCommand.TRANSFER;
    }
    public void startIntaking(){
        intakeCommand = IntakeCommand.INTAKE;
        intakeMode = IntakeMode.ANGLED;
    }
    public void angledIntake(){
        intakeCommand = IntakeCommand.INTAKE;
        intakeMode = IntakeMode.ANGLED;
    }
    private void setPositions(double rotation, double tilt, double gate) {
        rotationPosition = rotation;
        tiltPosition = tilt;
        gatePosition = gate;
    }
    public void verticalIntake(){
        intakeMode = IntakeMode.VERTICAL;
    }
    public void setColorToEject(String color){
        colorToEject = color;
    }
    public String getColorToEject(){
        return colorToEject;
    }
    public IntakeCommand getIntakeCommand(){
        return intakeCommand;
    }
    public AnalogInput currentRotationPosition(){
        return rotation;
    }

    public void logTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake Command", intakeCommand);
        telemetry.addData("Intake Mode", intakeMode);
        telemetry.addData("Slide Power", setSlidesPower);
        telemetry.addData("Sample Detected", sampleDetails());
        telemetry.addData("Rotation Position", rotation.getVoltage());
    }

}