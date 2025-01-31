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

    final double
            INTERMEDIATE_ROTATION = 0, RETRACT_LINKAGE = 0, RETRACT_LEFT_DIFF = 0, RETRACT_RIGHT_DIFF = 0,
            SPECIMEN_ROTATION = 0,SPECIMEN_LINKAGE = 0, SPECIMEN_LEFT_DIFF = 0, SPECIMEN_RIGHT_DIFF = 0,
            SPECIMEN_DEPOSIT_ROTATION = 0,SPECIMEN_DEPOSIT_LINKAGE = 0, SPECIMEN_DEPOSIT_LEFT_DIFF = 0, SPECIMEN_DEPOSIT_RIGHT_DIFF = 0,
            SAMPLE_DEPOSIT_ROTATION = 0, TRANSFER_LINKAGE = 0, TRANSFER_LEFT_DIFF = 0, TRANSFER_RIGHT_DIFF = 0,
            STANDBY_ROTATION = 0,STANDBY_LINKAGE = 0, STANDBY_LEFT_DIFF = 0, STANDBY_RIGHT_DIFF = 0;
    final double COUNTS_PER_REV_MOTOR = 384.5;
    double target,currentPos;
    final double ALLOWED_ERROR = 0.011;
    double SAMPLE_DEPOSIT = 3.8;
    double rotationPosition = 1, clawPosition = 0.9, linkagePosition = 0, leftDiffPosition = 0, rightDiffPosition = 0;
    PIDController slidesPID;
    ArrayList<Double> positionLog = new ArrayList<>();
    int posLogLength = 24;
    boolean colorSenor = false;
    boolean isIntakeTransferred;

    double p = 1.4,i = 0,d = 0,f = 0.2;
    //TODO: Set to false
    boolean pidTuning = false;
    double slidePower;
    String depositCommand = "retract";
    public  DepositV2(HardwareMap hardwareMap){
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //TODO: Set Configurations
        rotation = hardwareMap.get(ServoImplEx.class,"depositRotation");
        linkage = hardwareMap.get(ServoImplEx.class,"linkage");
        claw = hardwareMap.get(ServoImplEx.class,"claw");
        leftDifferential = hardwareMap.get(ServoImplEx.class,"lDiff");
        rightDifferential = hardwareMap.get(ServoImplEx.class,"rDiff");

        leftSlides = hardwareMap.get(DcMotorEx.class,"vLSlides");
        rightSlides = hardwareMap.get(DcMotorEx.class,"vRSlides");

        currentRotation = hardwareMap.get(AnalogInput.class, "currentDepositRotation");
        currentLinkage = hardwareMap.get(AnalogInput.class, "currentDepositLinkage");
        currentLeftDifferential = hardwareMap.get(AnalogInput.class, "currentDepositLeftDifferential");
        currentRightDifferential = hardwareMap.get(AnalogInput.class, "currentDepositRightDifferential");


        rotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        linkage.setPwmRange(new PwmControl.PwmRange(510,2490)); // Servo was spasing out  so we set to 510 and 2490 to make it start working. We spent too long on this peice of poopf
        claw.setPwmRange(new PwmControl.PwmRange(510,2490));
        leftDifferential.setPwmRange(new PwmControl.PwmRange(510,2490));
        rightDifferential.setPwmRange(new PwmControl.PwmRange(510,2490));

        linkage.setDirection(Servo.Direction.REVERSE);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        rotation.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: reset vars
        p = 1.75; i = 0; d = 0; f = 0.2;
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
                slidePower = -0.2;
                leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
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

        if(depositCommand.equals("transfer")){
            clawPosition = 1;
            if(isIntakeTransferred){
                rotationPosition = INTERMEDIATE_ROTATION;
            }
            if(currentRotation.getVoltage()>0){
                linkagePosition = TRANSFER_LINKAGE;
                leftDiffPosition = TRANSFER_LEFT_DIFF;
                rightDiffPosition = TRANSFER_RIGHT_DIFF;
                rotationPosition = SAMPLE_DEPOSIT_ROTATION;
            }
        }
        else if(depositCommand.equals("specimen")){
            if(currentRotation.getVoltage()>0){
                linkagePosition = SPECIMEN_LINKAGE;
                leftDiffPosition = SPECIMEN_LEFT_DIFF;
                rightDiffPosition = SPECIMEN_RIGHT_DIFF;
            }
            if(currentLeftDifferential.getVoltage()>0&&currentRightDifferential.getVoltage()>0){
                rotationPosition = SPECIMEN_ROTATION;
            }
        }
        else if(depositCommand.equals("depositSpecimen")){
            linkagePosition = SPECIMEN_DEPOSIT_LINKAGE;
            leftDiffPosition = SPECIMEN_DEPOSIT_LEFT_DIFF;
            rightDiffPosition = SPECIMEN_DEPOSIT_RIGHT_DIFF;
            rotationPosition = SPECIMEN_DEPOSIT_ROTATION;
        }
        else if(depositCommand.equals("retract")){
            rotationPosition = INTERMEDIATE_ROTATION;
            clawPosition = 0;
            if(currentRotation.getVoltage()>0) {
                linkagePosition = RETRACT_LINKAGE;
                if(currentLinkage.getVoltage()>0){
                    leftDiffPosition = RETRACT_LEFT_DIFF;
                    rightDiffPosition = RETRACT_RIGHT_DIFF;
                    if(currentLeftDifferential.getVoltage()>0&&currentRightDifferential.getVoltage()>0){
                        depositCommand = "standby";
                    }
                }
            }
        }
        else if(depositCommand.equals("standby")){
            if((colorSenor&&depositCommand.equals("standby"))){
                depositCommand = "transfer";
            }
            rotationPosition = STANDBY_ROTATION;
            clawPosition = 0;
            linkagePosition = STANDBY_LINKAGE;
            leftDiffPosition = STANDBY_LEFT_DIFF;
            rightDiffPosition = STANDBY_RIGHT_DIFF;
        }


    }


    public void specimenIntake(){
        rotationPosition = INTERMEDIATE_ROTATION;
        depositCommand = "specimen";
    }
    public void grabSpecimen(){
        clawPosition = 1;
    }
    public void depositSpecimen(){
        depositCommand = "depositSpecimen";
    }
    public void retract(){
        target = 0;
        depositCommand = "retract";
    }
    public void setSample(){
        target = SAMPLE_DEPOSIT;
        depositCommand = "depositSample";
    }
    public void releaseSample(){
        clawPosition = 0;
    }

    public String getDepositCommand(){
        return depositCommand;
    }
    public boolean slidesReachedTarget(){
        double avgRateChange1 = Math.abs(positionLog.get(posLogLength-1)-positionLog.get(0))/posLogLength;
        return (avgRateChange1 < ALLOWED_ERROR)&&Math.abs(target-currentPos)<ALLOWED_ERROR*15;
        //(leftSlides.getCurrent(CurrentUnit.AMPS)+rightSlides.getCurrent(CurrentUnit.AMPS))/2 > 7;
    }
    public void setTarget(double target){
        this.target = target;
    }
    public void setSenor(boolean bool){
        colorSenor = bool;
    }
    public void setIsIntakeTransferred(boolean bool){
        isIntakeTransferred = bool;
    }

}