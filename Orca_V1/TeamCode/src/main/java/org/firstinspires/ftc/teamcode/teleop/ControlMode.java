package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.OrcaV2;

@TeleOp(name = "TeleOp")
@Config

public class ControlMode extends OpMode {
    OrcaV2 orca;
    ElapsedTime runningTime = new ElapsedTime();
    ElapsedTime buzzTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double p,i,d,f,target;
    boolean intakeCross = false, intakeCircle = false, intakeTriangle = false, intakeSquare = false;
    /*
    public static double
            INTERMEDIATE_ROTATION = 0, RETRACT_LINKAGE = 0, RETRACT_LEFT_DIFF = 0, RETRACT_RIGHT_DIFF = 0,
    SPECIMEN_ROTATION = 0,SPECIMEN_LINKAGE = 0, SPECIMEN_LEFT_DIFF = 0, SPECIMEN_RIGHT_DIFF = 0,
    SPECIMEN_DEPOSIT_ROTATION = 0,SPECIMEN_DEPOSIT_LINKAGE = 0, SPECIMEN_DEPOSIT_LEFT_DIFF = 0.85, SPECIMEN_DEPOSIT_RIGHT_DIFF = 0,
    SAMPLE_DEPOSIT_ROTATION = 0, TRANSFER_LINKAGE = 0, TRANSFER_LEFT_DIFF = 0, TRANSFER_RIGHT_DIFF = 0,
    STANDBY_ROTATION = 0.6,STANDBY_LINKAGE = 0.15, STANDBY_LEFT_DIFF = 0.85, STANDBY_RIGHT_DIFF = 0.15;

    public static double
            ANGLED_ROTATION = 0.1, ANGLED_TILT = 0.15,
    VERTICAL_ROTATION = .11, VERTICAL_ROTATION_OFFSET = 0.25, VERTICAL_TILT = 0,
    EJECT_TILT = 0.5,
    TRANSFER_ROTATION = 1, TRANSFER_TILT = 0.95,
    intakeSTANDBY_ROTATION = 0.85, STANDBY_TILT = 0.6;

    public static String
            intakeCommand = "standby", depositCommand = "standby";
            */
    String currentAction = "intake";
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        orca = new OrcaV2(hardwareMap,new Pose(0,0,0));
        orca.teleopInit();
    }
    @Override
    public void init_loop(){
        telemetry.addLine("press cross for blue alliance, press triangle for red alliance");
        telemetry.addData("currently selected alliance: ", orca.intake().getColorToEject().equals("red") ? "blue" : "red");
        if(gamepad1.cross){
            orca.intake().setColorToEject("red");
        }
        else if(gamepad1.triangle){
            orca.intake().setColorToEject("blue");
        }
        //orca.deposit().PIDTuning(p,i,d,f,target);
        orca.intake().refresh(0,false,false,false,false,false);
        orca.deposit().refresh();
        telemetry.addData("Current Pos: ", orca.deposit().getCurrentSlidePosition());
        telemetry.addData("Target Pos: ", target);
        telemetry.update();
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
        telemetry.addData("Sample in intake: ", orca.intake().getCurrentSample());
        telemetry.addData("(Deposit) Current Pos: ", orca.deposit().getCurrentSlidePosition());
        telemetry.addData("(Deposit) Target Pos: ", target);
        telemetry.addData("(Intake) Current Pos: ", orca.intake().getCurrentSlidePosition());
        telemetry.addData("(Intake) Target Pos: ", orca.intake().getTargetSlidePosition());
        telemetry.addData("intake command: ", orca.intake().getIntakeCommand());
        telemetry.addData("deposit command: ", orca.deposit().getDepositCommand());
        telemetry.addData("current action: ", currentAction);
        telemetry.addData("intake rotation position: ", orca.intake().currentRotation().getVoltage());
        telemetry.addData("intake tilt position: ", orca.intake().currentTilt().getVoltage());
        telemetry.addData("deposit rotation position: ", orca.deposit().currentRotation().getVoltage());
        telemetry.addData("deposit linkage position: ", orca.deposit().getCurrentLinkage().getVoltage());
        telemetry.addData("deposit left diff position: ", orca.deposit().getCurrentLeftDifferential().getVoltage());
        telemetry.addData("deposit right diff position: ", orca.deposit().getCurrentRightDifferential().getVoltage());
        telemetry.update();




        if(gamepad1.options){
            currentAction = "sample";
        }
        else if(gamepad1.share){
            currentAction = "specimen";
            orca.deposit().specimenIntake();
            orca.intake().specimen();
        }
        else if(gamepad1.touchpad){
            orca.deposit().retract();
            currentAction = "intake";
        }

        if(orca.intake().getIntakeCommand().equals("transfer")){
            currentAction = "sample";
        }


        if(currentAction.equals("specimen")){
            if(gamepad1.right_bumper){
                orca.deposit().depositSpecimen();
            }
            else if(gamepad1.left_bumper){
                orca.deposit().specimenRetract();
                //currentAction = "intake";
            }
            else if(gamepad1.cross){
                orca.deposit().closeClaw();
            }
            else if(gamepad1.circle){
                orca.deposit().releaseClaw();
            }
            else if(gamepad1.square){
                orca.deposit().clipSpecimen();
            }
        }
        else if(currentAction.equals("sample")){
            if(gamepad1.right_bumper&&!orca.intake().getIntakeCommand().equals("transfer")){
                orca.deposit().setSample();
            }
            else if(gamepad1.left_bumper){
                orca.deposit().retract();
                currentAction = "intake";
            }
            else if(gamepad1.cross){
                orca.deposit().releaseClaw();
            }
        }

        if(currentAction.equals("intake")){
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
        }
        else {
            intakeCross = false;
            intakeCircle = false;
            intakeSquare = false;
            slidePower = 0;
        }
        intakeTriangle = gamepad1.triangle;

        orca.intake().refresh(slidePower,intakeCross,intakeCircle,intakeTriangle,intakeSquare,false);

        orca.deposit().refresh();

        orca.teleopRefresh(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);

        //TODO: remove readSampleDetails


    }
    @Override
    public void stop() {

    }
}