package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.OrcaV1;

@TeleOp(name = "TeleOp")
@Config
public class ControlMode extends OpMode{
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double p,i,d,f,target;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        orca = new OrcaV1(hardwareMap,new Pose2d(0,0,0));
    }
    @Override
    public void init_loop() {
        telemetry.addLine("Press cross for blue alliance, press triangle for red alliance");
        telemetry.addData("Currently selected alliance", orca.intake().getColorToEject().equals("red") ? "blue" : "red");

        if (gamepad1.cross) {
            orca.intake().setColorToEject("red");
        } else if (gamepad1.triangle) {
            orca.intake().setColorToEject("blue");
        }

        orca.deposit().PIDTuning(p, i, d, f, target);

        updateTelemetry();
    }
    @Override
    public void start(){
        runningTime.reset();
    }

    private void handleGamepadControls() {
        slidePower = gamepad1.right_trigger - gamepad1.left_trigger;

        if (gamepad1.circle) {
            orca.intake().angledIntake(); // 45-degree intake
        } else if (gamepad1.cross) {
            orca.intake().verticalIntake(); // Top-down intake
        }

        if (gamepad1.square) {
            orca.intake().retract();
        }

        if (gamepad1.right_trigger > 0) {
            orca.intake().startIntaking(); // Activate intake mode
        }

        if (gamepad1.dpad_up) {
            orca.deposit().setSample();
        } else if (gamepad1.dpad_left) {
            orca.deposit().setSpecimen();
        }
    }

    @Override
    public void loop() {
        handleGamepadControls();
        updateTelemetry();
        orca.refresh(gamepad1);
    }

    @Override
    public void stop() {

    }
    private void updateTelemetry() {
        //TODO: Remove read sample to fix loop times
        telemetry.addData("Sample in intake", orca.intake().readSampleDetails());
        telemetry.addData("Current Pos", orca.deposit().getCurrentPosition());
        telemetry.addData("Target Pos", target);
        telemetry.addData("Intake Command", orca.intake().getIntakeCommand());
        telemetry.addData("Deposit Command", orca.deposit().getDepositCommand());
        telemetry.addData("Rotation Position", orca.intake().currentRotationPosition().getVoltage());
        orca.logTelemetry(telemetry);
        telemetry.update();
    }
}