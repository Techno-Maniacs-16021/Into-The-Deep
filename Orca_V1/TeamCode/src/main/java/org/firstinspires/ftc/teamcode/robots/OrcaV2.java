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
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.mercurial.commands.Lambda;

public class OrcaV2 extends Follower {
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

    public OrcaV2(HardwareMap hardwareMap, Pose2d pose ) {
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        intake = new IntakeV2(hardwareMap);
        deposit = new DepositV2(hardwareMap);
    }
    public void teleopRefresh(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX){
        drive(gamepad1LeftStickX,gamepad1LeftStickY,gamepad1RightStickX);
        refresh();
    }
    public void refresh(){
        deposit.setSenor(intake.clawSenor());
        deposit.setIsIntakeTransferred(intake.isTransferred());
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
    public IntakeV2 intake(){
        return intake;
    }


    public DepositV2 deposit(){
        return deposit;
    }
    //auton pathing stuff
    public void deltaHeading(double degreesLeft) { // if you want to turn right, use negative degrees
        Pose temp = new Pose(this.getPose().getX(), this.getPose().getY(), this.getPose().getHeading() + Math.toRadians(degreesLeft));
        this.holdPoint(temp);
    }

    public void finalHeading(double degrees) { // if you want to turn right, use negative degrees
        Pose temp = new Pose(this.getPose().getX(), this.getPose().getY(), Math.toRadians(degrees));
        this.holdPoint(temp);
    }

    public Lambda turnTo(double degrees) {
        return new Lambda("follow-path")
                .addRequirements(this)
                .setInterruptible(true)
                .setInit(() -> this.finalHeading(degrees))
                .setExecute(() -> {
                    this.update();
                    //this.telemetryDebug(telemetry);

                })
                .setFinish(() -> !this.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) this.breakFollowing();
                });
    }

    public Lambda follow(Path path) {
        return new Lambda("follow-path")
                .addRequirements(this)
                .setInterruptible(true)
                .setInit(() -> this.followPath(path, true))
                .setExecute(() -> {
                    this.update();
                    //this.telemetryDebug(telemetry);

                })
                .setFinish(() -> !this.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) this.breakFollowing();
                });
    }


    //public DepositV2 deposit(){
        //return deposit;
    //}

}