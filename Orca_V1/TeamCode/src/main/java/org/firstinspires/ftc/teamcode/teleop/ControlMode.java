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
import org.firstinspires.ftc.teamcode.robots.OrcaV3;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.mercurial.Mercurial;

@TeleOp(name = "TeleOp")
@Config
@Mercurial.Attach
@BulkRead.Attach
@OrcaV3.Attach

public class ControlMode extends OpMode {
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
        OrcaV3.teleopInit();

    }
    @Override
    public void init_loop(){
        telemetry.addLine("press cross for blue alliance, press triangle for red alliance");
        telemetry.addData("currently selected alliance: ", OrcaV3.intake().getColorToEject().equals("red") ? "blue" : "red");
        if(gamepad1.cross){
            OrcaV3.intake().setColorToEject("red");
        }
        else if(gamepad1.triangle){
            OrcaV3.intake().setColorToEject("blue");
        }
        //orca.deposit().PIDTuning(p,i,d,f,target);
        telemetry.addData("Current Pos: ", OrcaV3.deposit().getCurrentSlidePosition());
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

        telemetry.addData("(Deposit) Current Pos: ", OrcaV3.deposit().getCurrentSlidePosition());
        telemetry.addData("(Deposit) Target Pos: ", target);
        telemetry.addData("(Intake) Current Pos: ", OrcaV3.intake().getCurrentSlidePosition());
        telemetry.addData("(Intake) Target Pos: ", OrcaV3.intake().getTargetSlidePosition());
        telemetry.addData("action: ", currentAction);
        telemetry.addData("intake command: ", OrcaV3.intake().getIntakeCommand());
        telemetry.addData("deposit command: ", OrcaV3.deposit().getDepositCommand());
        telemetry.addData("intake rotation position: ", OrcaV3.intake().currentRotation().getVoltage());
        telemetry.addData("intake tilt position: ", OrcaV3.intake().currentTilt().getVoltage());
        telemetry.addData("deposit rotation position: ", OrcaV3.deposit().currentRotation().getVoltage());
        telemetry.addData("deposit linkage position: ", OrcaV3.deposit().getCurrentLinkage().getVoltage());
        telemetry.addData("deposit left diff position: ", OrcaV3.deposit().getCurrentLeftDifferential().getVoltage());
        telemetry.addData("deposit right diff position: ", OrcaV3.deposit().getCurrentRightDifferential().getVoltage());
        telemetry.addData("colors:", OrcaV3.intake().colorPins() + " " + OrcaV3.deposit().colorPins());
        telemetry.update();




        if(gamepad1.options){
            currentAction = "sample";
        }
        else if(gamepad1.share){
            currentAction = "specimen";
            OrcaV3.deposit().specimenIntake();
            OrcaV3.intake().specimen();
        }
        else if(gamepad1.touchpad){
            OrcaV3.deposit().retract();
            currentAction = "intake";
        }

        if(OrcaV3.deposit().getDepositCommand().equals("depositSample")){
            currentAction = "sample";
        }


        if(currentAction.equals("specimen")){
            if(gamepad1.right_bumper){
                OrcaV3.deposit().depositSpecimen();
            }
            else if(gamepad1.left_bumper){
                OrcaV3.deposit().specimenRetract();
                //currentAction = "intake";
            }
            else if(gamepad1.cross){
                OrcaV3.deposit().closeClaw();
            }
            else if(gamepad1.circle){
                OrcaV3.deposit().releaseClaw();
            }
        }
        else if(currentAction.equals("sample")){
            if(Pasteurized.gamepad1().rightBumper().onTrue()){
                if(OrcaV3.deposit().checkCommand("depositHighSample")){
                    OrcaV3.deposit().setLowSample();
                }
                else{
                    OrcaV3.deposit().setSample();
                }
            }
            else if(gamepad1.left_bumper){
                OrcaV3.deposit().retract();
                currentAction = "intake";
            }
            else if(gamepad1.cross){
                OrcaV3.deposit().releaseClaw();
            }
        }

        if(currentAction.equals("intake")){
            if(gamepad1.circle){
                OrcaV3.intake().angledIntakeMode();
                //45* intake
            }
            else if(gamepad1.cross){
                OrcaV3.intake().verticalIntakeMode();
                //top down intake
            }
            if(gamepad1.dpad_left){
                OrcaV3.intake().transfer();
            }
            if (gamepad1.square){
                //retract
                OrcaV3.intake().retract();
            }

            if(gamepad1.right_trigger>0){
                OrcaV3.intake().startIntaking();
                //intake mode activate
            }
            //intakeCross = gamepad1.cross;
            //intakeCircle = gamepad1.circle;
            //intakeSquare = gamepad1.square;
            //intakeTriangle = gamepad1.triangle;
            //slidePower = gamepad1.right_trigger-gamepad1.left_trigger;
        }
        else {
            intakeCross = false;
            intakeCircle = false;
            intakeSquare = false;
            intakeTriangle = false;
            slidePower = 0;
        }

        OrcaV3.teleopRefresh(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);

        //TODO: remove readSampleDetails


    }
    @Override
    public void stop() {

    }
}