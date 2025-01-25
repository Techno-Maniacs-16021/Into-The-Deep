package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.tuning.localization.PinpointDrive;

public class OrcaV1 extends PinpointDrive{
    //IntakeV1 INTAKE;
    //DepositV1 DEPOSIT;
    private final IntakeV1 intake;
    private final DepositV1 deposit;
    public OrcaV1(HardwareMap hardwareMap, Pose2d pose ) {
        super(hardwareMap, pose);
        intake = new IntakeV1(hardwareMap);
        deposit = new DepositV1(hardwareMap);
    }
    public void refresh(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX){

        drive(gamepad1LeftStickX,gamepad1LeftStickY,gamepad1RightStickX);
    }
    public void drive(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX){
        this.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1LeftStickY,
                        -gamepad1LeftStickX
                ),
                -gamepad1RightStickX
        ));
    }
    public IntakeV1 intake(){
        return intake;
    }
    public DepositV1 deposit(){
        return deposit;
    }
    public void specimenInit(){
        deposit.specimenInit();
        intake.init();
        intake.refresh(0,false,false,false,false,false);
        deposit.refresh();
    }
    public void sampleInit(){
        deposit.retract();
        deposit.sampleInit();
        intake.init();
        intake.refresh(0,false,false,false,false,false);
        deposit.refresh();
    }



}