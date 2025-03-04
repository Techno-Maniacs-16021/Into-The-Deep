package org.firstinspires.ftc.teamcode.robots;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;

public class IntakeV3 {
    DigitalChannel colorPin0,colorPin1,colorPin2,colorPin3,colorPin4,colorPin5;
    //servos
    ServoImplEx tilt, rotation, gate;
    //motor
    DcMotorEx intake, slides;

    AnalogInput currentRotation, currentTilt;
    PIDController slidesPID;

    RevBlinkinLedDriver lightBar;

    public double
            ANGLED_ROTATION = 0.15, ANGLED_TILT = 0.225,
            VERTICAL_ROTATION = 0.2 , VERTICAL_ROTATION_OFFSET = 0.4, VERTICAL_TILT = 0.05,
            EJECT_TILT = 0.5,
            SPECIMEN_ROTATION = 0.86, SPECIMEN_TILT = 0.715,
            TRANSFER_ROTATION = 0.86, TRANSFER_TILT = 0.715,
            STANDBY_ROTATION = 0.86, STANDBY_TILT = 0.715,
            INTAKE_DEPLOY_OFFSET = 1;

    double rotationPosition = STANDBY_ROTATION, tiltPosition = STANDBY_TILT, gatePosition = 0;

    final double COUNTS_PER_REV_MOTOR = 145.1*(2/1);
    final double STATIC_INTAKE_POWER = 0;
    final double INTAKE_EJECT_SLIDES_OFFSET = 3;
    //TODO: set false
    final boolean pidTuning = false;
    boolean isPIDActive = false;
    boolean isIntakeMotorActive = false;
    double target,currentPos;
    double ALLOWED_ERROR = 0.0125;
    double p = 0.0,i = 0,d = 0,f=0;
    int globalTime = 0;
    boolean recentlyEjected = false;
    String intakeCommand = "standby", intakeMode = "angled";
    String colorToEject = "red";
    String sampleColor = "none";
    double slidesPower = 0, intakePower = 0;
    int nSensorSamples = 20;
    ArrayList<String> colorSensorInputs = new ArrayList<String>();

    int posLogLength = 12;
    ArrayList<Double> positionLog = new ArrayList<>();

    ElapsedTime transferTimer = new ElapsedTime();

    boolean intakeButton = Pasteurized.gamepad1().a().state()||Pasteurized.gamepad1().b().state();
    boolean reverseIntakeButton = Pasteurized.gamepad1().y().state();
    double intakeSlidesTrigger = Pasteurized.gamepad1().rightTrigger().state()-Pasteurized.gamepad1().leftTrigger().state();
    public IntakeV3(HardwareMap hardwareMap) {

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //TODO: set configurations
        colorPin0 = hardwareMap.digitalChannel.get("crf0");
        colorPin1 = hardwareMap.digitalChannel.get("crf1");
        colorPin2 = hardwareMap.digitalChannel.get("crf2");
        colorPin3 = hardwareMap.digitalChannel.get("crf3");
        colorPin4 = hardwareMap.digitalChannel.get("crf4");
        colorPin5 = hardwareMap.digitalChannel.get("crf5");


        tilt = hardwareMap.get(ServoImplEx.class,"tilt");
        rotation = hardwareMap.get(ServoImplEx.class,"introt");
        gate = hardwareMap.get(ServoImplEx.class,"gate");

        intake = hardwareMap.get(DcMotorEx.class,"intake");
        slides = hardwareMap.get(DcMotorEx.class,"hSlides");

        currentRotation = hardwareMap.get(AnalogInput.class, "cintrot");
        currentTilt = hardwareMap.get(AnalogInput.class, "ctilt");

        lightBar = hardwareMap.get(RevBlinkinLedDriver.class,"lightBar");

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

    public void refresh (){
        if(sampleColor.equals("red")){
            lightBar.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if(sampleColor.equals("blue")){
            lightBar.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        else if(sampleColor.equals("yellow")){
            lightBar.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        else{
            lightBar.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }

        intakeSlidesTrigger = Pasteurized.gamepad1().rightTrigger().state()-Pasteurized.gamepad1().leftTrigger().state();
        intakeButton = Pasteurized.gamepad1().a().state()||Pasteurized.gamepad1().b().state();
        reverseIntakeButton = Pasteurized.gamepad1().y().state();

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

        slideControlLoop();
        intakeModuleControlLoop();

        sampleColor = readSampleDetails();
    }

    public String readSampleDetails() {
        /*
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
         */
        //System.out.println();
        boolean red,blue;
        blue =colorPin2.getState();
        red =colorPin3.getState();
        return (
                blue && red ? "yellow"
                        : !blue && red ? "red"
                        : blue && !red ? "blue"
                        : "none"
        );
    }
    /*public void updateSampleDetails(){
        boolean red,blue;
        //System.out.println(colorPin0.getState()+" "+colorPin2.getState()+" "+ colorPin1.getState()+" "+colorPin3.getState()+" "+ colorPin4.getState()+" "+colorPin5.getState());
        blue =colorPin2.getState();
        red =colorPin3.getState();
        colorSensorInputs.add(
                blue && red ? "yellow"
                        : !blue && red ? "red"
                        : blue && !red ? "blue"
                        : "none");
        colorSensorInputs.remove(0);
    }*/
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

    public void slideControlLoop(){
        if(intakeCommand.equals("init")||intakeCommand.equals("specimen")){
            if ((slides.getCurrent(CurrentUnit.AMPS) < 7||currentPos>0.25) && this.slidesPower != -0.25) {
                this.slidesPower = -1;
            }
            else {
                if (Math.abs(currentPos) > 0.05) {
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
                if (Math.abs(currentPos) > 0.05) {
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
        else if(isPIDActive){
            slidesPID.setPID(p, i, d);
            this.slidesPower = slidesPID.calculate(currentPos, target)+f;
        }
        else {
            this.slidesPower = intakeSlidesTrigger;
        }
    }
    public void intakeModuleControlLoop(){

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
                else if(isIntakeMotorActive||intakeButton){
                    intakePower = 1;
                    if(currentPos>INTAKE_DEPLOY_OFFSET) {
                        rotationPosition = VERTICAL_ROTATION;
                        tiltPosition = VERTICAL_TILT;
                    }
                }
                else if(reverseIntakeButton){
                    intakePower = -1;
                    if(currentPos>INTAKE_DEPLOY_OFFSET) {
                        rotationPosition = VERTICAL_ROTATION_OFFSET;
                        tiltPosition = EJECT_TILT;
                    }
                }
                else{
                    intakePower = STATIC_INTAKE_POWER;
                    if(currentPos>INTAKE_DEPLOY_OFFSET||rotationPosition!=STANDBY_ROTATION){
                        rotationPosition = VERTICAL_ROTATION_OFFSET;
                        tiltPosition = VERTICAL_TILT;
                    }
                }
            }
            else{
                gatePosition = 0;
                //intake flat on floor(45 degrees)
                //rotation set to lower angle
                //does not move up or down when intaking
                if(isIntakeMotorActive||intakeButton){
                    intakePower = 1;
                }
                else if(reverseIntakeButton){
                    intakePower = -1;
                }
                else{
                    intakePower = STATIC_INTAKE_POWER;
                }
                if(colorEject()!=0&&currentPos<INTAKE_EJECT_SLIDES_OFFSET){
                    tiltPosition = EJECT_TILT;
                }
                else if(currentPos>INTAKE_DEPLOY_OFFSET||rotationPosition!=STANDBY_ROTATION){
                    tiltPosition = ANGLED_TILT;
                }
                if(currentPos>INTAKE_DEPLOY_OFFSET||rotationPosition!=STANDBY_ROTATION) {
                    rotationPosition = ANGLED_ROTATION;
                }
            }
            //when button pressed, intake on. When button not pressed, intake off.
            //auto sample ejection
        }
        else if(intakeCommand.equals("retract")){
            if(reverseIntakeButton) {
                intakePower = -1;
            }
            else if(currentRotation.getVoltage()>2.16-0.15&&currentTilt.getVoltage()<1.58+0.15&&!sampleColor.equals("none") && !sampleColor.equals(colorToEject)){
                gatePosition = 1;
                intakePower = 1;
            }
            else {
                intakePower = STATIC_INTAKE_POWER;
                rotationPosition = STANDBY_ROTATION;
                tiltPosition = STANDBY_TILT;
            }
        }
        else if(intakeCommand.equals("standby")){
            rotationPosition = STANDBY_ROTATION;
            tiltPosition = STANDBY_TILT;
            intakePower = STATIC_INTAKE_POWER;
            gatePosition = 1;
        }
        else if(intakeCommand.equals("transfer")){
            if(colorPin0.getState()||colorPin1.getState()||colorPin4.getState()||colorPin5.getState()){
                transferTimer.reset();
                intakeCommand = "transferred";
            }
            gatePosition = 1;
            intakePower = 1;
        }
        else if(intakeCommand.equals("transferred")){
            if(transferTimer.milliseconds()>1000){
                gatePosition = 1;
                intakePower = 0;
                intakeCommand = "standby";
            }
            else if(transferTimer.milliseconds()>500){
                rotationPosition = TRANSFER_ROTATION;
                tiltPosition = TRANSFER_TILT;
                intakePower = 0.5;
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
        gatePosition = 0;
    }
    public void specimen(){
        intakeCommand = "specimen";
    }
    public void angledIntakeMode(){
        intakeMode = "angled";
    }

    public void verticalIntakeMode(){
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
    public void enablePID(){
        isPIDActive = true;
    }
    public void disablePID(){
        isPIDActive = false;
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
    public void setIntakeCommand(String command){
        intakeCommand = command;
    }

    public boolean checkCommand(String targetCommand){
        return targetCommand.equals(intakeCommand);
    }

    public void enableIntakeMotor(){
        isIntakeMotorActive = true;
    }
    public void disableIntakeMotor(){
        isIntakeMotorActive = false;
    }
    public String colorPins(){
        return "0:" + colorPin0.getState() + " 1:" + colorPin1.getState() + " 2:" + colorPin2.getState() + " 3:" + colorPin3.getState();
    }


}
