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
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

public class IntakeV2 {
    DigitalChannel colorPin0,colorPin1,colorPin2,colorPin3,colorPin4,colorPin5;
    //servos
    ServoImplEx tilt, rotation, gate;
    //motor
    DcMotorEx intake, slides;

    AnalogInput currentRotation, currentTilt;
    PIDController slidesPID;


    public double
            ANGLED_ROTATION = 0.05, ANGLED_TILT = 0.15,
            VERTICAL_ROTATION = 0.1, VERTICAL_ROTATION_OFFSET = 0.25, VERTICAL_TILT = 0,
            EJECT_TILT = 0.5,
            SPECIMEN_ROTATION = 0.4, SPECIMEN_TILT = 0.6,
    TRANSFER_ROTATION = 0.65, TRANSFER_TILT = 0.6,
            STANDBY_ROTATION = 0.85, STANDBY_TILT = 0.6;
    double rotationPosition = STANDBY_ROTATION, tiltPosition = STANDBY_TILT, gatePosition = 0;

    final double COUNTS_PER_REV_MOTOR = 145.1*(2/1);
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
    boolean color3,isTransferred,isTransferring;
    String intakeCommand = "standby", intakeMode = "angled";
    String colorToEject = "red";
    String sampleColor = "none";
    double slidesPower = 0, intakePower = 0;
    int nSensorSamples = 20;
    ArrayList<String> colorSensorInputs = new ArrayList<String>();

    int posLogLength = 12;
    ArrayList<Double> positionLog = new ArrayList<>();

    ElapsedTime transferTimer = new ElapsedTime();
    public IntakeV2(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //TODO: set configurations
        colorPin0 = hardwareMap.digitalChannel.get("crf0");
        colorPin1 = hardwareMap.digitalChannel.get("crf1");
        colorPin2 = hardwareMap.digitalChannel.get("crf2");
        colorPin3 = hardwareMap.digitalChannel.get("crf3");
        colorPin4 = hardwareMap.digitalChannel.get("crf4");
        colorPin5 = hardwareMap.digitalChannel.get("crf5");


        tilt = hardwareMap.get(ServoImplEx.class,"intakeTilt");
        rotation = hardwareMap.get(ServoImplEx.class,"intakeRotation");
        gate = hardwareMap.get(ServoImplEx.class,"gate");

        intake = hardwareMap.get(DcMotorEx.class,"intake");
        slides = hardwareMap.get(DcMotorEx.class,"hSlides");

        currentRotation = hardwareMap.get(AnalogInput.class, "cir");
        currentTilt = hardwareMap.get(AnalogInput.class, "cit");

        tilt.setPwmRange(new PwmControl.PwmRange(510,2490));
        rotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        //gate.setPwmRange(new PwmControl.PwmRange(510,2490));

        tilt.setDirection(ServoImplEx.Direction.REVERSE);
        gate.setDirection(ServoImplEx.Direction.REVERSE);

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

        transferTimer.reset();
    }

    public void refresh (double slidesPower,boolean verticalIntake, boolean angledIntake, boolean reverseIntake, boolean retract, boolean PID){
        isPIDActive = PID;

        tilt.setPosition(tiltPosition);
        rotation.setPosition(rotationPosition);
        slides.setPower(this.slidesPower);

        currentPos = slides.getCurrentPosition()/COUNTS_PER_REV_MOTOR;
        positionLog.add(currentPos);
        positionLog.remove(0);

        //TODO: Fix color rejection
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
                    //intake.setPower(REVERSE_INTAKE_POWER_TILT);
                }
                else{
                    intake.setPower(colorEject());
                    gate.setPosition(1);
                }
            }
        }
        /*else if(recentlyEjected){
            resetColorSamples();
            recentlyEjected = false;
        }*/
        else {
            intake.setPower(this.intakePower);
            gate.setPosition(gatePosition);
        }

        slideControlLoop(slidesPower,retract, PID);
        intakeModuleControlLoop(verticalIntake,angledIntake,reverseIntake);

        updateSampleDetails();
        sampleColor = readSampleDetails();

        color3 = colorPin4.getState()||colorPin5.getState();

    }

    public String readSampleDetails() {
        double blue = 0, red = 0, yellow = 0, none = 0;
        for(int k = 0; k<nSensorSamples;k++){
            String color = colorSensorInputs.get(k);
            if(color.equals("blue"))
                blue+=(k+1.0)/(nSensorSamples);
            else if(color.equals("red"))
                red+=(k+1.0)/(nSensorSamples);
            else if(color.equals("yellow"))
                yellow+=(k+1.0)/(nSensorSamples);
            else
                none+=(k+1.0)/(nSensorSamples);
            //System.out.print(color+" ");
        }
        System.out.println();
        return (
                blue>red && blue>yellow &&  blue>none ? "blue"
                        : red>blue && red>yellow && red>none ? "red"
                        : none>red && none>blue && none>yellow ? "none"
                        //: (intakeCommand.equals("transfer")&&none>(0.75*nSensorSamples)) ? "none"
                        : "yellow"
        );
    }
    public void updateSampleDetails(){
        boolean red,blue;
        //System.out.println(colorPin0.getState()+" "+colorPin2.getState()+" "+ colorPin1.getState()+" "+colorPin3.getState()+" "+ colorPin4.getState()+" "+colorPin5.getState());
        blue =colorPin0.getState()||colorPin2.getState();
        red =colorPin1.getState()||colorPin3.getState();
        colorSensorInputs.add(
                blue && red ? "yellow"
                        : !blue && red ? "red"
                        : blue && !red ? "blue"
                        : "none");
        colorSensorInputs.remove(0);
    }
    public String getCurrentSample(){
        return sampleColor;
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
        if(intakeCommand.equals("init")||intakeCommand.equals("specimen")){
            if ((slides.getCurrent(CurrentUnit.AMPS) < 7||currentPos>0.25) && this.slidesPower != -0.25) {
                this.slidesPower = -1;
            }
            else {
                if (Math.abs(currentPos) < 0.05) {
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

                if (!sampleColor.equals("none") && !sampleColor.equals(colorToEject)) {
                    intakeCommand = "transfer";
                    //TODO: remove when servo is fixed
                    //intakePower = REVERSE_INTAKE_POWER_TILT;
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
            rotationPosition = STANDBY_ROTATION;
            tiltPosition = STANDBY_TILT;
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
            if(color3||sampleColor.equals("none")){
                transferTimer.reset();
                intakeCommand = "transferred";
                isTransferring = true;
            }
            gatePosition = 1;
            intakePower = 1;
        }
        else if(intakeCommand.equals("transferred")){
            isTransferring = true;
            if(transferTimer.milliseconds()>1600){
                gatePosition = 0;
                intakePower = 0;
                isTransferred = false;
                isTransferring = false;
                intakeCommand = "standby";
            }
            else if(transferTimer.milliseconds()>800){
                rotationPosition = TRANSFER_ROTATION;
                tiltPosition = TRANSFER_TILT;
                intakePower = 0.5;
                isTransferred = true;
            }
        }
        else if(intakeCommand.equals("specimen")){
            intakePower = 0;
            tiltPosition = SPECIMEN_TILT;
            rotationPosition = SPECIMEN_ROTATION;

        }
        else if(intakeCommand.equals("init")){
            intakePower = 0;
            tiltPosition = 0.9;
            rotationPosition = 0.75;

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
    public void specimen(){
        intakeCommand = "specimen";
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
    public boolean clawSenor(){
        return color3;
    }
    public boolean isTransferred(){
        return isTransferred;
    }
    public boolean isTransferring(){
        return isTransferring;
    }
    public void setIntakeCommand(String command){
        intakeCommand = command;
    }


}
