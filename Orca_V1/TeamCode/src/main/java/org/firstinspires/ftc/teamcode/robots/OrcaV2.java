package org.firstinspires.ftc.teamcode.robots;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.groups.Parallel;

public class OrcaV2 {
    //IntakeV1 INTAKE;
    //DepositV1 DEPOSIT;
    private final IntakeV2 intake;
    private final DepositV2 deposit;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    HardwareMap hardwareMap;

    private String state = "Standby";
    Follower follower;

    public OrcaV2(HardwareMap hardwareMap, Pose startPose) {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        this.hardwareMap = hardwareMap;
        intake = new IntakeV2(hardwareMap);
        deposit = new DepositV2(hardwareMap);
    }
    public void teleopRefresh(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX){
        follower.setTeleOpMovementVectors(-gamepad1LeftStickY, -gamepad1LeftStickX, -gamepad1RightStickX);
        deposit.setDepositCommand("specimen");
        refresh();
    }
    public void refresh(){
        deposit.setSenor(intake.isTransferring());
        deposit.setIsIntakeTransferred(intake.isTransferred());
        deposit.setIsIntakeTransferring(intake.isTransferring());
        follower.update();
    }
    public void teleopInit (){
        follower.startTeleopDrive();
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void autoInit (){
        deposit.setDepositCommand(" ");
        deposit.autoINIT();
        deposit.refresh();
        intake.setIntakeCommand("standby");
        intake.refresh(0,false,false,false,false,false);
    }
    public IntakeV2 intake(){
        return intake;
    }


    public DepositV2 deposit(){
        return deposit;
    }


    //auton pathing stuff

    public void turnTo(double radians) { // if you want to turn right, use negative degrees
        Pose temp = new Pose(follower.getPose().getX(), follower.getPose().getY(), radians);
        follower.holdPoint(temp);
    }

    public Lambda turn(double radians) {
        return new Lambda("turn")
                .addRequirements(follower)
                .setInterruptible(true)
                .setExecute(() -> {
                    follower.update();
                    this.turnTo(radians);
                    //this.telemetryDebug(telemetry);

                })
                .setFinish(() -> !follower.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    public Lambda follow(Path path) {
        return new Lambda("follow-path")
                .addRequirements(follower)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(path, true))
                .setExecute(() -> {
                    follower.update();
                    //this.telemetryDebug(telemetry);

                })
                .setFinish(() -> !follower.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    public Follower getFollower() {
        return follower;
    }

}