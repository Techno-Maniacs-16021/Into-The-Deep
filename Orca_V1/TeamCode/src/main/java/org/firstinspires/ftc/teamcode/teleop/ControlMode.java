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
import org.firstinspires.ftc.teamcode.robots.OrcaV2;

@TeleOp(name = "TeleOp")
@Config
public class ControlMode extends OpMode{
    OrcaV2 orca;
    ElapsedTime runningTime = new ElapsedTime();
    ElapsedTime buzzTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double p,i,d,f,target;
    boolean intakeCross = false, intakeCircle = false, intakeTriangle = false, intakeSquare = false;

    String currentAction = "intake";
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        orca = new OrcaV2(hardwareMap,new Pose2d(0,0,0));
    }
    @Override
    public void init_loop(){
        telemetry.addLine("press cross for blue alliance, press triangle for red alliance");
        telemetry.addData("currently selected alliance: ", orca.intake().getColorToEject().equals("red") ? "blue" : "red");
        telemetry.update();
        if(gamepad1.cross){
            orca.intake().setColorToEject("red");
        }
        else if(gamepad1.triangle){
            orca.intake().setColorToEject("blue");
        }
        /*orca.deposit().PIDTuning(p,i,d,f,target);
        orca.deposit().refresh(true);
        telemetry.addData("Current Pos: ", orca.deposit().getCurrentPosition());
        telemetry.addData("Target Pos: ", target);
        telemetry.update();*/
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }
    @Override
    public void start(){
        //orca.sampleInit();
        runningTime.reset();
        buzzTime.reset();
    }
    @Override
    public void loop() {
        if(gamepad1.circle){
            orca.intake().angledIntake();
            //45* intake
        }
        else if(gamepad1.cross){
            orca.intake().verticalIntake();
            //top down intake
        }
        if(gamepad1.dpad_left){
            orca.intake().transfer();
        }
        if (gamepad1.square){
            //retract
            orca.intake().retract();
        }

        if(gamepad1.right_trigger>0){
            orca.intake().startIntaking();
            //intake mode activate
        }

        intakeCross = gamepad1.cross;
        intakeCircle = gamepad1.circle;
        intakeSquare = gamepad1.square;
        slidePower = gamepad1.right_trigger-gamepad1.left_trigger;

        orca.intake().refresh(slidePower,intakeCross,intakeCircle,intakeTriangle,intakeSquare,false);

        //TODO: remove readSampleDetails


    }
    @Override
    public void stop() {

    }
}