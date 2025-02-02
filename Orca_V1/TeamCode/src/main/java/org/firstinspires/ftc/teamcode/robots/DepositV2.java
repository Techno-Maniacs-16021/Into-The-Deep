package org.firstinspires.ftc.teamcode.robots;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

public class DepositV2{
    ServoImplEx rotation, claw, linkage, leftDifferential, rightDifferential;
    DcMotorEx leftSlides,rightSlides;
    AnalogInput currentRotation, currentLinkage, currentLeftDifferential, currentRightDifferential;

    public double
            INTERMEDIATE_ROTATION = 0.3, RETRACT_LINKAGE = 0.15, RETRACT_LEFT_DIFF = 0.85, RETRACT_RIGHT_DIFF = 0.15,
            SPECIMEN_ROTATION = 0.0,SPECIMEN_LINKAGE = 1, SPECIMEN_LEFT_DIFF = 0, SPECIMEN_RIGHT_DIFF = 0.3,
            SPECIMEN_DEPOSIT_ROTATION = 0,SPECIMEN_DEPOSIT_LINKAGE = 1, SPECIMEN_DEPOSIT_LEFT_DIFF = 0.0, SPECIMEN_DEPOSIT_RIGHT_DIFF = 0.3,SPECIMEN_DEPOSIT_CLIP_LEFT_DIFF = 0.0, SPECIMEN_DEPOSIT_CLIP_RIGHT_DIFF = 0.3,
            //SPECIMEN_DEPOSIT_ROTATION = 1,SPECIMEN_DEPOSIT_LINKAGE = 0, SPECIMEN_DEPOSIT_LEFT_DIFF = 0.0, SPECIMEN_DEPOSIT_RIGHT_DIFF = 0.3,SPECIMEN_DEPOSIT_CLIP_LEFT_DIFF = 0.6, SPECIMEN_DEPOSIT_CLIP_RIGHT_DIFF = 0.0,
            SAMPLE_DEPOSIT_ROTATION = 0.5, TRANSFER_LINKAGE = 1, TRANSFER_LEFT_DIFF = 0.7, TRANSFER_RIGHT_DIFF = 0,
            STANDBY_ROTATION = .145,STANDBY_LINKAGE = 0.15, STANDBY_LEFT_DIFF = 0.85, STANDBY_RIGHT_DIFF = 0.15;


    final double COUNTS_PER_REV_MOTOR = 384.5;
    final double ALLOWED_SERVO_ERROR = 0.15;
    double target,currentPos;
    final double ALLOWED_ERROR = 0.011;
    double SAMPLE_DEPOSIT = 2.3, SPECIMEN_PRIME = 1.4, SPECIMEN = 0.5;
    double rotationPosition = 1, clawPosition = 0.9, linkagePosition = 0, leftDiffPosition = 0, rightDiffPosition = 0;
    PIDController slidesPID;
    ArrayList<Double> positionLog = new ArrayList<>();
    int posLogLength = 24;
    boolean colorSenor = false;
    boolean isIntakeTransferred,isIntakeTransferring;

    double p = 4,i = 0,d = 0,f = 0.2;
    //TODO: Set to false
    boolean pidTuning = false;
    double slidePower;
    String depositCommand = "standby";
    public  DepositV2(HardwareMap hardwareMap){
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //TODO: Set Configurations
        rotation = hardwareMap.get(ServoImplEx.class,"depositRotation");
        linkage = hardwareMap.get(ServoImplEx.class,"depositLinkage");
        claw = hardwareMap.get(ServoImplEx.class,"claw");
        leftDifferential = hardwareMap.get(ServoImplEx.class,"lDiff");
        rightDifferential = hardwareMap.get(ServoImplEx.class,"rDiff");

        leftSlides = hardwareMap.get(DcMotorEx.class,"vLSlides");
        rightSlides = hardwareMap.get(DcMotorEx.class,"vRSlides");

        currentRotation = hardwareMap.get(AnalogInput.class, "cdr");
        currentLinkage = hardwareMap.get(AnalogInput.class, "cdl");
        currentLeftDifferential = hardwareMap.get(AnalogInput.class, "cdld");
        currentRightDifferential = hardwareMap.get(AnalogInput.class, "cdrd");


        rotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        linkage.setPwmRange(new PwmControl.PwmRange(510,2490)); // Servo was spasing out  so we set to 510 and 2490 to make it start working. We spent too long on this peice of poopf
        claw.setPwmRange(new PwmControl.PwmRange(510,2490));
        leftDifferential.setPwmRange(new PwmControl.PwmRange(510,2490));
        rightDifferential.setPwmRange(new PwmControl.PwmRange(510,2490));

        //linkage.setDirection(Servo.Direction.REVERSE);
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        //rotation.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);


        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: reset vars
        p = 4; i = 0; d = 0; f = 0.2;
        slidesPID = new PIDController(p,i,d);

        for(int k = 0; k < posLogLength; k++){
            positionLog.add(0.0);
        }
    }
    public void refresh(){
        leftSlides.setPower(slidePower);
        rightSlides.setPower(slidePower);

        currentPos = (leftSlides.getCurrentPosition()+rightSlides.getCurrentPosition())/(2*COUNTS_PER_REV_MOTOR);
        positionLog.add(currentPos);
        positionLog.remove(0);

        controlLoop();
        slidesLoop();


        rotation.setPosition(rotationPosition);
        claw.setPosition(clawPosition);
        linkage.setPosition(linkagePosition);
        rightDifferential.setPosition(rightDiffPosition);
        leftDifferential.setPosition(leftDiffPosition);

    }
    public void slidesLoop(){
        if(target != 0){
            slidesPID.setPID(p,i,d);
            slidePower = slidesPID.calculate(currentPos,target)+f;
        }
        else{
            if((leftSlides.getCurrent(CurrentUnit.AMPS)+rightSlides.getCurrent(CurrentUnit.AMPS))/2 < 7&&currentPos>0.1){
                slidePower = -1;
            }
            else{
                slidePower = -0.3;
                if(currentPos>ALLOWED_ERROR*10&&currentPos<0.2){
                    leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
    }
    public double getCurrentSlidePosition(){
        return currentPos;
    }
    public boolean slidesReachedTarget(){
        double avgRateChange1 = Math.abs(positionLog.get(posLogLength-1)-positionLog.get(0))/posLogLength;
        return (avgRateChange1 < ALLOWED_ERROR)&&Math.abs(target-currentPos)<ALLOWED_ERROR*15;
        //(leftSlides.getCurrent(CurrentUnit.AMPS)+rightSlides.getCurrent(CurrentUnit.AMPS))/2 > 7;
    }
    public void setTarget(double target){
        this.target = target;
    }
    public void PIDTuning (double p, double i, double d, double f, double target) {
        if(pidTuning) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.target = target;
        }
    }
    public void controlLoop(){

        if(depositCommand.equals("transfer")||depositCommand.equals("depositSample")){
            if(!colorSenor&&depositCommand.equals("transfer")){
                //depositCommand = "standby";
            }
            if(isIntakeTransferred){
                rotationPosition = INTERMEDIATE_ROTATION;
            }
            if(currentRotation.getVoltage()<(2+ALLOWED_SERVO_ERROR)){
                linkagePosition = TRANSFER_LINKAGE;
                leftDiffPosition = TRANSFER_LEFT_DIFF;
                rightDiffPosition = TRANSFER_RIGHT_DIFF;
                rotationPosition = SAMPLE_DEPOSIT_ROTATION;
                depositCommand = "depositSample";
            }
        }
        else if(depositCommand.equals("specimen")){
            if(currentRotation.getVoltage()<(2+ALLOWED_SERVO_ERROR)){
                linkagePosition = SPECIMEN_LINKAGE;
                leftDiffPosition = SPECIMEN_LEFT_DIFF;
                rightDiffPosition = SPECIMEN_RIGHT_DIFF;
            }
            if((Math.abs(0.18-currentLeftDifferential.getVoltage())<ALLOWED_SERVO_ERROR)&&(Math.abs(2.28-currentRightDifferential.getVoltage())<ALLOWED_SERVO_ERROR)){
                rotationPosition = SPECIMEN_ROTATION;
            }
        }
        else if(depositCommand.equals("depositSpecimen")||depositCommand.equals("depositSpecimenClip")){
            linkagePosition = SPECIMEN_DEPOSIT_LINKAGE;
            if(depositCommand.equals("depositSpecimen")){
                leftDiffPosition = SPECIMEN_DEPOSIT_LEFT_DIFF;
                rightDiffPosition = SPECIMEN_DEPOSIT_RIGHT_DIFF;
            }
            else{
                leftDiffPosition = SPECIMEN_DEPOSIT_CLIP_LEFT_DIFF;
                rightDiffPosition = SPECIMEN_DEPOSIT_CLIP_RIGHT_DIFF;
            }
            if(currentPos<0.7&&target==SPECIMEN){
                clawPosition = 0;
            }
            rotationPosition = SPECIMEN_DEPOSIT_ROTATION;
        }
        else if(depositCommand.equals("retract")){
            rotationPosition = INTERMEDIATE_ROTATION;
            clawPosition = 0;
            if(Math.abs(2.07-currentRotation.getVoltage())<ALLOWED_SERVO_ERROR) {
                linkagePosition = RETRACT_LINKAGE;
                if(Math.abs(2.03-currentLinkage.getVoltage())<ALLOWED_SERVO_ERROR){
                    leftDiffPosition = RETRACT_LEFT_DIFF;
                    rightDiffPosition = RETRACT_RIGHT_DIFF;
                    if((Math.abs(2.71-currentLeftDifferential.getVoltage())<ALLOWED_SERVO_ERROR)&&(Math.abs(2.73-currentRightDifferential.getVoltage())<ALLOWED_SERVO_ERROR)){
                        depositCommand = "standby";
                    }
                }
            }
        }
        else if(depositCommand.equals("testing")){
            linkagePosition = SPECIMEN_DEPOSIT_LINKAGE;
            leftDiffPosition = SPECIMEN_DEPOSIT_LEFT_DIFF;
            rightDiffPosition = SPECIMEN_DEPOSIT_RIGHT_DIFF;
            rotationPosition = SPECIMEN_DEPOSIT_ROTATION;
        }
        else if(depositCommand.equals("standby")){
            if((colorSenor)||isIntakeTransferring){
                clawPosition = 1;
                claw.setPosition(clawPosition);
                depositCommand = "transfer";
            }
            else{
                clawPosition = 0;
            }
            rotationPosition = STANDBY_ROTATION;
            linkagePosition = STANDBY_LINKAGE;
            leftDiffPosition = STANDBY_LEFT_DIFF;
            rightDiffPosition = STANDBY_RIGHT_DIFF;
        }


    }


    public void specimenIntake(){
        rotationPosition = INTERMEDIATE_ROTATION;
        depositCommand = "specimen";
        target = 0;
    }
    public void closeClaw(){
        clawPosition = 1;
    }
    public void depositSpecimen(){
        depositCommand = "depositSpecimen";
        target = SPECIMEN_PRIME;
    }
    public void clipSpecimen(){
        target = SPECIMEN;
    }
    public void specimenRetract(){
        target = 0;
        depositCommand = "specimen";
    }
    public void retract(){
        target = 0;
        depositCommand = "retract";
    }
    public void setSample(){
        target = SAMPLE_DEPOSIT;
        depositCommand = "depositSample";
    }
    public void releaseClaw(){
        clawPosition = 0;
    }

    public String getDepositCommand(){
        return depositCommand;
    }
    public void setSenor(boolean bool){
        colorSenor = bool;
    }
    public void setIsIntakeTransferred(boolean bool){
        isIntakeTransferred = bool;
    }
    public void setIsIntakeTransferring(boolean bool){
        isIntakeTransferring = bool;
    }
    public AnalogInput currentRotation(){
        return currentRotation;
    }
    public AnalogInput getCurrentLinkage(){
        return currentLinkage;
    }
    public AnalogInput getCurrentLeftDifferential(){
        return currentLeftDifferential;
    }
    public AnalogInput getCurrentRightDifferential(){
        return currentRightDifferential;
    }
    public void setDepositCommand(String command){
        depositCommand = command;
    }
    public void autoINIT(){
        rotationPosition = SPECIMEN_ROTATION;
        linkagePosition = 0.3;
        leftDiffPosition = SPECIMEN_LEFT_DIFF;
        rightDiffPosition = SPECIMEN_RIGHT_DIFF;
        clawPosition = 1;

    }

}