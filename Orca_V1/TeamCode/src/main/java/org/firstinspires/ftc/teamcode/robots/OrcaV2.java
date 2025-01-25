package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class OrcaV2 extends Follower {
    //IntakeV1 INTAKE;
    //DepositV1 DEPOSIT;
    //private final IntakeV1 intake;
    //private final DepositV1 deposit;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    HardwareMap hardwareMap;

    private String state = "Standby";

    public OrcaV2(HardwareMap hardwareMap, Pose2d pose ) {
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        //intake = new IntakeV1(hardwareMap);
        //deposit = new DepositV1(hardwareMap);
    }
    public void teleopRefresh(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX){
        drive(gamepad1LeftStickX,gamepad1LeftStickY,gamepad1RightStickX);
        refresh();
    }
    public void refresh(){
        
    }

    public void drive(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX){
        this.setTeleOpMovementVectors(-gamepad1LeftStickY, -gamepad1LeftStickX, -gamepad1RightStickX);
        this.update();
    }
    public void teleopInit (){
        this.startTeleopDrive();
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /*public IntakeV1 intake(){
        return intake;
    }
     */
    /*
    public DepositV1 deposit(){
        return deposit;
    }
     */
}