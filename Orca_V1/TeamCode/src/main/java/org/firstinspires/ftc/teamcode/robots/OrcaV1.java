package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.DepositV1;
import org.firstinspires.ftc.teamcode.components.IntakeV1;
import org.firstinspires.ftc.teamcode.localization.MecanumDrive;

public class OrcaV1 extends MecanumDrive{
    //IntakeV1 INTAKE;
    //DepositV1 DEPOSIT;
    private final IntakeV1 intake;
    private final DepositV1 deposit;
    public OrcaV1(HardwareMap hardwareMap, Pose2d pose ) {
        super(hardwareMap, pose);

        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }

        intake = new IntakeV1(hardwareMap);
        deposit = new DepositV1(hardwareMap);
    }

    public void refresh(Gamepad gamepad1){
        drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        intake.refresh(gamepad1.right_trigger-gamepad1.left_trigger, gamepad1.cross, gamepad1.circle, gamepad1.triangle);
        deposit.refresh(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.dpad_right,gamepad1.dpad_down);
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

    public void logTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive Pose", this.pose);
        telemetry.addData("Intake Command", intake.getIntakeCommand());
        telemetry.addData("Deposit Command", deposit.getDepositCommand());
        intake.logTelemetry(telemetry);
        deposit.logTelemetry(telemetry);
    }
}