package org.firstinspires.ftc.teamcode.robots;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DepositV1{
    ServoImplEx clawRotation, specimenClaw, depositLinkage;
    DcMotorEx leftSlides,rightSlides;
    final double COUNTS_PER_REV_MOTOR = 384.5;
    double target,currentPos;
    double SAMPLE_DEPOSIT = 3.9, SPECIMEN_DEPOSIT_PRIME = 0, SPECIMEN_DEPOSIT = 0;
    double clawRotationPosition = 0, clawPosition = 0, depositPosition = 0;
    PIDController slidesPID;


    double p = 1.25,i = 0,d = 0,f = 0.2;
    //TODO: Set to false
    boolean pidTuning = false;
    double slidePower;
    String depositCommand = "retract";
    public  DepositV1(HardwareMap hardwareMap){
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //TODO: Set Configurations
        clawRotation = hardwareMap.get(ServoImplEx.class,"clawRotation");
        depositLinkage = hardwareMap.get(ServoImplEx.class,"linkage");
        specimenClaw = hardwareMap.get(ServoImplEx.class,"claw");

        leftSlides = hardwareMap.get(DcMotorEx.class,"vLSlides");
        rightSlides = hardwareMap.get(DcMotorEx.class,"vRSlides");

        clawRotation.setPwmRange(new PwmControl.PwmRange(510,2490));
        depositLinkage.setPwmRange(new PwmControl.PwmRange(510,2490)); // Servo was spasing out  so we set to 510 and 2490 to make it start working. We spent too long on this peice of poopf
        specimenClaw.setPwmRange(new PwmControl.PwmRange(510,2490));

        depositLinkage.setDirection(Servo.Direction.REVERSE);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: reset vars

        slidesPID = new PIDController(p,i,d);
    }
    public void refresh(boolean leftBumper, boolean rightBumper, boolean dpadRight, boolean dpadDown){
        leftSlides.setPower(slidePower);
        rightSlides.setPower(slidePower);
        currentPos = (leftSlides.getCurrentPosition()+rightSlides.getCurrentPosition())/(2*COUNTS_PER_REV_MOTOR);
        controlLoop(leftBumper,rightBumper,dpadRight,dpadDown);
        PIDLoop();
        clawRotation.setPosition(clawRotationPosition);
        specimenClaw.setPosition(clawPosition);
        depositLinkage.setPosition(depositPosition);

    }
    public void PIDLoop(){
        if(!depositCommand.equals("retract")){
            slidesPID.setPID(p,i,d);
            slidePower = slidesPID.calculate(currentPos,target)+f;
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
    public void controlLoop(boolean leftBumper, boolean rightBumper, boolean dpadRight, boolean dpadDown){
        if(depositCommand.equals("sample")){
            if(rightBumper){
                depositPosition = 1;
            }
            else if(leftBumper){
                resetDeposit();
                depositCommand = "retract";
            }
        }
        else if(depositCommand.equals("specimen")){
            if(dpadDown){
                target = SPECIMEN_DEPOSIT;
            }
            if(leftBumper){
                resetDeposit();
                depositCommand = "retract";
            }

        }
        else{
            if((leftSlides.getCurrent(CurrentUnit.AMPS)+rightSlides.getCurrent(CurrentUnit.AMPS))/2 < 7&&currentPos>0.1){
                slidePower = -1;
            }
            else{
                slidePower = -0.2;
            }
            if(dpadRight){
                clawRotationPosition = 1;
                System.out.println("button pressed");
            }
            if(rightBumper){
                clawPosition = 1;
            }
        }

    }

    public void resetDeposit(){
        depositPosition = 0;
        clawPosition = 0 ;
        clawRotationPosition = 0;
    }
    public void setSample(){
        depositCommand = "sample";
        target = SAMPLE_DEPOSIT;
    }
    public void setSpecimen(){
        depositCommand = "specimen";
        target = SPECIMEN_DEPOSIT_PRIME;
    }
    public double getCurrentPosition(){
        return currentPos;
    }
    public void retract(){

    }
    public String getDepositCommand(){
        return depositCommand + ": " + depositPosition;
    }
}