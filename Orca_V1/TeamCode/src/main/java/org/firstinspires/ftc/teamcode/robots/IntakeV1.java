package org.firstinspires.ftc.teamcode.robots;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    PIDController slidesPID;


    final double ANGLED_ROTATION = 0.1, ANGLED_TILT = 0.3,
            VERTICAL_ROTATION = .11, VERTICAL_ROTATION_OFFSET = 0.25, VERTICAL_TILT = 0.15,
            RETRACT_ROTATION = 0.25, RETRACT_TILT = .6,
            EJECT_TILT = 0.5,
            TRANSFER_ROTATION = 1, INTER_TRANSFER_ROTATION = 0.7, TRANSFER_TILT = 0.95,
            STANDBY_ROTATION = 0.7, STANDBY_TILT = 0.6;
    double rotationPosition = STANDBY_ROTATION, tiltPosition = STANDBY_TILT, gatePosition = 0;

    final double COUNTS_PER_REV_MOTOR = 145.1*(2/1);
    final double REVERSE_INTAKE_POWER_TILT = -0.2;
    final double STATIC_INTAKE_POWER = 0.4;
    final double INTAKE_EJECT_SLIDES_OFFSET = 3;
    //TODO: set false
    final boolean pidTuning = false;
    boolean isPIDActive = false;
    double target,currentPos;
    double ALLOWED_ERROR = 0.0125;
    double p = 0.0,i = 0,d = 0,f=0;

    int globalTime = 0;
    boolean recentlyEjected = false;
    String intakeCommand = "standby", intakeMode = "angled";
    String colorToEject = "red";
    String sampleColor = "none";
    double slidesPower = 0, intakePower = 0;
    int nSensorSamples = 40;
    ArrayList<String> colorSensorInputs = new ArrayList<String>();

    int posLogLength = 12;
    ArrayList<Double> positionLog = new ArrayList<>();

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

        //tilt.setDirection(Servo.Direction.REVERSE);
        //gate.setDirection(Servo.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //slides.setDirection(DcMotorSimple.Direction.REVERSE);

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TODO: reset vars
        target = 0; currentPos = 0;
        intakeCommand = "standby";
        intakeMode = "angled";

        p = 2;i = 0.0;d = 0.1;f=0.2;
        slidesPID = new PIDController(p,i,d);

        for(int k = 0; k < nSensorSamples; k++){
            colorSensorInputs.add("none");
        }
        for(int k = 0; k < posLogLength; k++){
            positionLog.add(0.0);
        }
    }
    public void refresh (double slidesPower,boolean verticalIntake, boolean angledIntake, boolean reverseIntake, boolean retract, boolean PID){
        isPIDActive = PID;
        tilt.setPosition(tiltPosition);
        rotation.setPosition(rotationPosition);
        slides.setPower(this.slidesPower);
        currentPos = slides.getCurrentPosition()/COUNTS_PER_REV_MOTOR;
        positionLog.add(currentPos);
        positionLog.remove(0);
        if(colorEject()<0){
            intake.setPower(colorEject());
            recentlyEjected = true;
        }
        else if(colorEject()>0){
            if(intakeMode.equals("angled")){
                gate.setPosition(1);
                intake.setPower(colorEject());
                recentlyEjected = true;
            }
            else {
                //TODO: fix when servo is fixed
                if(currentTilt.getVoltage()>1.95){
                    intake.setPower(REVERSE_INTAKE_POWER_TILT);
                }
                else{
                    intake.setPower(colorEject());
                    gate.setPosition(1);
                }
            }
        }
        else if(recentlyEjected){
            resetColorSamples();
            recentlyEjected = false;
        }
        else {
            intake.setPower(this.intakePower);
            gate.setPosition(gatePosition);
        }

        slideControlLoop(slidesPower,retract, PID);
        intakeModuleControlLoop(verticalIntake,angledIntake,reverseIntake);
        updateSampleDetails();
        sampleColor = readSampleDetails();


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

            //System.out.print(color+" ");
        }
        System.out.println();
        return (
                blue>nSensorSamples*0.8 ? "blue"
                        : red>nSensorSamples*0.8 ? "red"
                        : none>nSensorSamples*0.8 ? "none"
                        //: (intakeCommand.equals("transfer")&&none>(0.75*nSensorSamples)) ? "none"
                        : "yellow"
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
    public String getCurrentColorReading(){
        return colorPin0.getState() && colorPin1.getState() ? "yellow"
                : !colorPin0.getState() && colorPin1.getState() ? "red"
                : colorPin0.getState() && !colorPin1.getState() ? "blue"
                : "none";
    }
    public String getCurrentSample(){
        return sampleColor;
    }
    private void resetColorSamples(){
        for(int k = 0; k < nSensorSamples; k++){
            colorSensorInputs.remove(0);
            colorSensorInputs.add("none");
        }
    }
    public double colorEject(){
        //maybe use position instead
        if(currentPos<INTAKE_EJECT_SLIDES_OFFSET&&sampleColor.equals(colorToEject)){
            return -0.5;
        }
        /*else if(intakeMode.equals("vertical")&&sampleColor.equals(colorToEject)){
            return (1);
        }*/
        else if(sampleColor.equals(colorToEject)){
            return (1);
        }
        return 0.0;
    }
    public void slideControlLoop(double slidesPower, boolean retract, boolean PID){
        if(intakeCommand.equals("init")){
            if ((slides.getCurrent(CurrentUnit.AMPS) < 7||currentPos>0.25) && this.slidesPower != -0.25) {
                this.slidesPower = -1;
            }
            else {
                if (Math.abs(currentPos) < 0.01) {
                    slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                currentPos = 0;
                this.slidesPower = -0.25;
            }
        }
        else if (intakeCommand.equals("retract") || intakeCommand.equals("standby")) {
            if ((slides.getCurrent(CurrentUnit.AMPS) < 7||currentPos>0.25) && this.slidesPower != -0.25) {
                this.slidesPower = -1;
            }
            else {
                if (Math.abs(currentPos) < 0.01) {
                    slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                currentPos = 0;

                this.slidesPower = -0.25;

                if ((!sampleColor.equals("none") && !sampleColor.equals(colorToEject) && retract)||sampleColor.equals("yellow")) {
                    intakeCommand = "transfer";
                    //TODO: remove when servo is fixed
                    intakePower = REVERSE_INTAKE_POWER_TILT;
                }
                else if (sampleColor.equals("none")) {
                    intakeCommand = "standby";
                }
            }
        }
        else if (intakeCommand.equals("transfer")) {
            this.slidesPower = -0.25;
        }
        else if(PID){
            slidesPID.setPID(p, i, d);
            this.slidesPower = slidesPID.calculate(currentPos, target)+f;
        }
        else {
            this.slidesPower = slidesPower;
        }
    }
    public void intakeModuleControlLoop(boolean verticalIntake, boolean angledIntake, boolean reverseIntake){
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
                if(colorEject()!=0){
                    rotationPosition = VERTICAL_ROTATION_OFFSET;
                    tiltPosition = EJECT_TILT;
                }
                else if(verticalIntake){
                    intakePower = 1;
                    rotationPosition = VERTICAL_ROTATION;
                    tiltPosition = VERTICAL_TILT;
                }
                else if(reverseIntake){
                    intakePower = -1;
                    rotationPosition = VERTICAL_ROTATION_OFFSET;
                    tiltPosition = EJECT_TILT;
                }
                else{
                    intakePower = STATIC_INTAKE_POWER;
                    rotationPosition = VERTICAL_ROTATION_OFFSET;
                    tiltPosition = VERTICAL_TILT;
                }
            }
            else{
                gatePosition = 0;
                //intake flat on floor(45 degrees)
                //rotation set to lower angle
                //does not move up or down when intaking
                if(angledIntake){
                    intakePower = 1;
                }
                else if(reverseIntake){
                    intakePower = -1;
                }
                else{
                    intakePower = STATIC_INTAKE_POWER;
                }
                if(colorEject()!=0&&currentPos<INTAKE_EJECT_SLIDES_OFFSET){
                    tiltPosition = EJECT_TILT;
                }
                else {
                    tiltPosition = ANGLED_TILT;
                }
                rotationPosition = ANGLED_ROTATION;
            }
            //when button pressed, intake on. When button not pressed, intake off.
            //auto sample ejection
        }
        else if(intakeCommand.equals("retract")){
            rotationPosition = RETRACT_ROTATION;
            tiltPosition = RETRACT_TILT;
            if(reverseIntake) {
                intakePower = -1;
            }
            else {
                intakePower = STATIC_INTAKE_POWER;
            }
        }
        else if(intakeCommand.equals("standby")){
            rotationPosition = STANDBY_ROTATION;
            tiltPosition = STANDBY_TILT;
            intakePower = STATIC_INTAKE_POWER;
        }
        else if(intakeCommand.equals("transfer")){
            if(currentTilt.getVoltage()>1.25&&rotationPosition != TRANSFER_ROTATION){
                rotationPosition = INTER_TRANSFER_ROTATION;
            }
            else{
                rotationPosition = TRANSFER_ROTATION;
            }
            tiltPosition = TRANSFER_TILT;
            if(currentRotation.getVoltage()<1.2&&!sampleColor.equals("none")&&!reverseIntake){
                gatePosition = 1;
                intakePower = 1;
            }
            else if(sampleColor.equals("none")||reverseIntake){
                gatePosition = 0;
                intakePower = 0;
                for(int k = 0; k < nSensorSamples; k++){
                    colorSensorInputs.remove(0);
                    colorSensorInputs.add("none");
                }
                intakeCommand = "standby";
            }
        }
        else if(intakeCommand.equals("init")){
            intakePower = 0;
            tiltPosition = 0.9;
            rotationPosition = 0.75;

        }
    }

    public void makeArmGoDowny(){

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
    public double getCurrentSlidePosition(){
        return currentPos;
    }
    public double getTargetSlidePosition(){
        return target;
    }
    public void setTargetSlidePosition(double pos){
        target = pos;
    }
    public boolean slidesReachedTarget(){
        double avgRateChange1 = Math.abs(positionLog.get(posLogLength-1)-positionLog.get(0))/posLogLength;
        return (!isPIDActive&&slides.getCurrent(CurrentUnit.AMPS) > 7)||
                (isPIDActive&&(avgRateChange1 < ALLOWED_ERROR)&&Math.abs(target-currentPos)<ALLOWED_ERROR*25);

    }
    public void PIDTuning (double p, double i, double d, double f,double target) {
        if(pidTuning) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.target = target;
        }
    }
    public void setTarget(double target){
        this.target = target;
    }

    public double getRotationVoltage(){
        return currentRotation.getVoltage();
    }
    public boolean isEjecting(){
        return recentlyEjected;
    }
    public void init(){
        intakeCommand = "init";
    }



}