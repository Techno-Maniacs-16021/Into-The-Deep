package org.firstinspires.ftc.teamcode.robots;

import static java.lang.Thread.sleep;

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
    final double ALLOWED_ERROR = 0.03;
    double SAMPLE_DEPOSIT = 3.9, SPECIMEN_DEPOSIT_PRIME = 2.1, SPECIMEN_DEPOSIT = 0.8;
    double clawRotationPosition = 1, clawPosition = 0.9, depositPosition = 0;
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
        clawRotation.setDirection(Servo.Direction.REVERSE);
        specimenClaw.setDirection(Servo.Direction.REVERSE);

        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: reset vars
        p = 1.25; i = 0; d = 0; f = 0.2;
        slidesPID = new PIDController(p,i,d);
    }
    public void refresh(){
        leftSlides.setPower(slidePower);
        rightSlides.setPower(slidePower);
        currentPos = (leftSlides.getCurrentPosition()+rightSlides.getCurrentPosition())/(2*COUNTS_PER_REV_MOTOR);
        controlLoop();
        PIDLoop();
        clawRotation.setPosition(clawRotationPosition);
        specimenClaw.setPosition(clawPosition);
        depositLinkage.setPosition(depositPosition);

    }
    public void PIDLoop(){
        if(target!=0){
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
    public void controlLoop(){
        if(target==0){
            if((leftSlides.getCurrent(CurrentUnit.AMPS)+rightSlides.getCurrent(CurrentUnit.AMPS))/2 < 7&&currentPos>0.1){
                slidePower = -1;
            }
            else{
                slidePower = -0.2;
            }
        }
        if(target == SPECIMEN_DEPOSIT && currentPos<1.3 )
            clawPosition = 0.2;

    }



    public void setSample(){
        depositCommand = "sample";
        target = SAMPLE_DEPOSIT;
        //depositPosition = 0.65;
    }
    public void specimenIntake(){
        depositCommand = "specimen";
        clawPosition = 0;
        clawRotationPosition = 1;
    }
    public void closeClaw(){
        clawPosition = 0.9;
    }
    public void scoreSpecimen(){
        target = SPECIMEN_DEPOSIT;
    }
    public void setSpecimen(){
        target = SPECIMEN_DEPOSIT_PRIME;
    }
    public double getCurrentSlidePosition(){
        return currentPos;
    }
    public void retract(){
        depositPosition = 0;
        clawPosition = 0.2 ;
        clawRotationPosition = 0;
        depositCommand = "retract";
        target = 0;
    }
    public void depositSample(){
        depositPosition = 1;
    }
    public void resetBucket(){
        depositPosition = 0;
    }


    public String getDepositCommand(){
        return depositCommand;
    }
    public boolean slidesReachedTarget(){
        return (Math.abs(target - currentPos) < ALLOWED_ERROR)||
        (leftSlides.getCurrent(CurrentUnit.AMPS)+rightSlides.getCurrent(CurrentUnit.AMPS))/2 > 7;
    }
    public void setTarget(double target){
        this.target = target;
    }

}