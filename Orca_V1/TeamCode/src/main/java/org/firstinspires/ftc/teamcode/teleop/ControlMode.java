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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.IntakeV1;
import org.firstinspires.ftc.teamcode.robots.OrcaV1;

import java.util.List;

@TeleOp(name = "TeleOp")
@Config
public class ControlMode extends OpMode{
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();

    @Override
    public void init() {
        orca = new OrcaV1(hardwareMap,new Pose2d(0,0,0));
    }
    @Override
    public void init_loop(){
//
    }
    @Override
    public void start(){
        runningTime.reset();
    }
    @Override
    public void loop() {
        telemetry.addData("Sample in intake: ", orca.intake.sampleDetails());
        telemetry.update();
        if(gamepad1.cross)
            orca.intake.neutralPosition();
        if(gamepad1.dpad_up)
            orca.intake.tiltUp((int) runningTime.milliseconds());
        else if(gamepad1.dpad_down)
            orca.intake.tiltDown((int) runningTime.milliseconds());

        if(gamepad1.dpad_right)
            orca.intake.rotateForward((int) runningTime.milliseconds());
        else if(gamepad1.dpad_left)
            orca.intake.rotateBackward((int) runningTime.milliseconds());

        if(gamepad1.right_trigger!=0)
            orca.intake.intake(gamepad1.right_trigger);
        else if(!(intake.sampleDetails().equals("red")))
            orca.intake.intake(0);

        orca.intake.updateLoop();
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