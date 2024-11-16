package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.robots.IntakeV1;
import org.firstinspires.ftc.teamcode.robots.OrcaV1;

import java.util.List;

@TeleOp(name = "TeleOp")
@Config
public class ControlMode extends OpMode{
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double p,i,d,f,target;
    @Override
    public void init() {
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
        orca.deposit().PIDTuning(p,i,d,f,target);

    }
    @Override
    public void start(){
        runningTime.reset();
    }
    @Override
    public void loop() {
        telemetry.addData("Sample in intake: ", orca.intake().sampleDetails());
        telemetry.addData("Current Pos: ", orca.deposit().getCurrentPosition());
        telemetry.addData("Target Pos: ", target);
        telemetry.addData("intake command: ", orca.intake().getIntakeCommand());
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
        Servo servo = hardwareMap.get(ServoImplEx.class,"depositLinkage");
        servo.setPosition(0);
        orca.intake().refresh(slidePower,gamepad1.cross,gamepad1.circle,gamepad1.triangle);
        orca.deposit().refresh(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.dpad_right,gamepad1.dpad_down);
        orca.refresh(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        slidePower = gamepad1.right_trigger-gamepad1.left_trigger;

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
        if(gamepad1.dpad_up){
            orca.deposit().setSample();
        }
        else if(gamepad1.dpad_left){
            orca.deposit().setSpecimen();
        }
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

    }
    @Override
    public void stop() {

    }
}