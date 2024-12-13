package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.localization.Drawing;
import org.firstinspires.ftc.teamcode.robots.IntakeV1;
import org.firstinspires.ftc.teamcode.robots.OrcaV1;

import java.util.List;

@TeleOp(name = "TeleOp")
@Config
public class ControlMode extends OpMode{
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    ElapsedTime buzzTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double p,i,d,f,target;
    boolean intakeCross = false, intakeCircle = false, intakeTriangle = false, intakeSquare = false;

    String currentAction = "intake";
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        orca = new OrcaV1(hardwareMap,new Pose2d(0,0,0));
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
        runningTime.reset();
        buzzTime.reset();
    }
    @Override
    public void loop() {
        //TODO: remove readSampleDetails
        telemetry.addData("Sample in intake: ", orca.intake().getCurrentSample());
        telemetry.addData("(Deposit) Current Pos: ", orca.deposit().getCurrentSlidePosition());
        telemetry.addData("(Deposit) Target Pos: ", target);
        telemetry.addData("(Intake) Current Pos: ", orca.intake().getCurrentSlidePosition());
        telemetry.addData("(Intake) Target Pos: ", orca.intake().getTargetSlidePosition());
        telemetry.addData("intake command: ", orca.intake().getIntakeCommand());
        telemetry.addData("deposit command: ", orca.deposit().getDepositCommand());
        telemetry.addData("current action: ", currentAction);
        telemetry.addData("rotation position: ", orca.intake().currentRotation().getVoltage());
        telemetry.addData("tilt position: ", orca.intake().currentTilt().getVoltage());
        telemetry.update();
        /*
        if(gamepad1.cross)
            orca.intake().neutralPosition();
        if(gamepad1.dpad_up)
            orca.intake().tiltUp((int) runningTime.milliseconds());
        else if(gamepad1.dpad_down)
            orca.intake().tiltDown((int) runningTime.milliseconds());

        if(gamepad1.dpad_right)
            orca.intake().rotateForward((int) runningTime.milliseconds());
        else if(gamepad1.dpad_left)
            orca.intake().rotateBackward((int) runningTime.milliseconds());

        if(gamepad1.right_trigger!=0)
            orca.intake().intakeSample(gamepad1.right_trigger);
        else if(!(orca.intake().sampleDetails().equals("red")))
            orca.intake().intakeSample(0);
        */

        if(gamepad1.options){
            currentAction = "sample";
        }
        else if(gamepad1.share){
            currentAction = "specimen";
            orca.deposit().specimenIntake();
            orca.intake().retract();
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
                orca.deposit().setSpecimen();
            }
            else if(gamepad1.left_bumper){
                orca.deposit().retract();
                currentAction = "intake";
            }
            else if(gamepad1.cross){
                orca.deposit().closeClaw();
            }
            else if(gamepad1.circle){
                orca.deposit().scoreSpecimen();
            }
        }
        else if(currentAction.equals("sample")){
            if(gamepad1.right_bumper){
                orca.deposit().setSample();
            }
            else if(gamepad1.left_bumper){
                orca.deposit().retract();
                currentAction = "intake";
            }
            else if(gamepad1.cross){
                orca.deposit().depositSample();
            }
            else if(gamepad1.circle){
                orca.deposit().resetBucket();
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
            intakeTriangle = gamepad1.triangle;
            intakeSquare = gamepad1.square;
            slidePower = gamepad1.right_trigger-gamepad1.left_trigger;
        }
        else {
            intakeCross = false;
            intakeCircle = false;
            intakeTriangle = false;
            intakeSquare = false;
            slidePower = 0;
        }

        orca.intake().refresh(slidePower,intakeCross,intakeCircle,intakeTriangle,intakeSquare,false);

        if(currentAction.equals("specimen"))
            orca.deposit().refresh(false);
        else
            orca.deposit().refresh(true);

        orca.refresh(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        //sample intake
            //intake flat on floor(45 degrees)
                //rotation set to lower angle
                //does not move up or down when intaking
            //intake vertically(90 degrees)
                //rotation set to a higher angle
                //moves down when intaking
                //returns to higher position when not intaking
            //General
                //move horizontal slides back and forth
                //color sensor for intake detection
                //auto retraction and transfer on detection
                    //rotation and tilt up to neutral angle
                    //slides back to 0 position
                    //rotation and tilt back to transfer angle
                    //open gate and intake on
                    //close gate and intake off
                    //return rotation and tilt to neutral
                    //set to deposit sample mode
                //Red and Blue alliance mode to spit out wrong samples

        //specimen intake
            //slides to intake position
            //claw flips out
            //one side of claw rotates to align specimen
            //close claw when specimen in range
            //set to deposit specimen mode

        //deposit
            //sample
                //up to top basket height
                //bucket action on button press
                //auto retract bucket and slides
                //set to intake mode
            //specimen
                //up to high chambers height
                //down to specimen clip height on button press
                //claw open, flip claw down, and retract slides
                //set to intake mode
        String allianceColor = orca.intake().getColorToEject().equals("red") ? "blue" : "red";
        if(orca.intake().getCurrentSample().equals("yellow")&&buzzTime.milliseconds()>300){
            gamepad1.rumble(50);
            buzzTime.reset();
        }
        else if(orca.intake().getCurrentSample().equals(allianceColor)){
            gamepad1.rumbleBlips(1);
        }


    }
    @Override
    public void stop() {

    }
}