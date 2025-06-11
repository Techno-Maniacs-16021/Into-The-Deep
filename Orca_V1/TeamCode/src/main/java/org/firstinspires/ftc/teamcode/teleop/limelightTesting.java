package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.pathgen.Path;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.pathing.Paths;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auton.pathing.Paths;
import org.firstinspires.ftc.teamcode.robots.OrcaV3;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

@TeleOp
@Config
@Mercurial.Attach
@BulkRead.Attach
@OrcaV3.Attach
public class limelightTesting extends OpMode {

    private static Limelight3A limelight;
    public static double defaultError = 1.5;
    private Telemetry telemetryA;
    boolean tele = false;


    @Override
    public void init() {
        //claw = hardwareMap.get(ServoImplEx.class,"claw");
        //claw.setDirection(Servo.Direction.REVERSE);
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        OrcaV3.teleopInit();
        Paths.init();
        limelight = FeatureRegistrar.getActiveOpMode().hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    @Override
    public void init_loop(){
        //servo0.setPosition(0);
        //servo1.setPosition(1);
    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {
        if(tele)
            OrcaV3.follower().telemetryDebug(telemetryA);


        if (Pasteurized.gamepad1().cross().onTrue()) {
            Pose current = getCurrentLPose();
            tele = true;
            OrcaV3.follower().breakFollowing();
            OrcaV3.follower().setStartingPose(current);
            Path limelightPath = new Path(
                    new BezierCurve(
                            new Point(current),
                            Paths.clearSubCurve,
                            new Point(Paths.leadInPause),
                            new Point(Paths.pause)
                    )
            );
            limelightPath.setConstantHeadingInterpolation(Paths.pause.getHeading());

            Path specCollectPath = new Path(
                    new BezierLine(
                            new Point(Paths.pause),
                            new Point(Paths.specCollect)
                    )
            );
            /*
            PathChain autoBucketTo =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Point(follower.getPose()),
                                            new Point(58.000, 119.000),
                                            new Point(autoBucketToEndPose)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autoBucketToEndPose.getHeading())
                            .build();
             */

            specCollectPath.setConstantHeadingInterpolation(Paths.specCollect.getHeading());

            PathChain fullPath = OrcaV3.follower().pathBuilder().addPath(limelightPath).addPath(specCollectPath).build();

            new Parallel(
                    new Sequential(
                            OrcaV3.follow(limelightPath, false,4*defaultError),
                            OrcaV3.follow(specCollectPath, false,defaultError,6)
                            //OrcaV3.follow(Paths.specPathMap.get("align-Spec-Curve"),Paths.specPathMap.get("collect-Spec"),false,defaultError)
                    )
            ).schedule();
        }


        if (Pasteurized.gamepad1().circle().onTrue()) {
            tele = false;
            OrcaV3.follower().breakFollowing();
            OrcaV3.follower().setStartingPose(new Pose(0,0,0));
            //OrcaV3.follower().startTeleopDrive();
        }
    }

    public Pose getCurrentLPose(){
        Pose current = new Pose(0,0,0);
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                double xOffset;
                double yOffset;
                double headingOffset;

                if(result.getFiducialResults().get(0).getFiducialId()==15){
                    xOffset = 3; // 48+45+3
                    yOffset = -3; //
                    headingOffset = Math.PI*0.5; //180
                    current = new Pose(botpose.getPosition().y*-39.37+xOffset,botpose.getPosition().x*39.37+yOffset,botpose.getOrientation().getYaw(AngleUnit.RADIANS)+headingOffset);
                }
                else{
                    xOffset = 3;
                    yOffset = -3;
                    headingOffset = Math.PI*-0.5;
                    current = new Pose(botpose.getPosition().y*39.37+xOffset,-1*botpose.getPosition().x*39.37+yOffset,botpose.getOrientation().getYaw(AngleUnit.RADIANS)+headingOffset);

                }
            }
        }
        else{
            current = OrcaV3.follower().getPose();
        }

        return current;
    }
    @Override
    public void stop() {

    }

}
