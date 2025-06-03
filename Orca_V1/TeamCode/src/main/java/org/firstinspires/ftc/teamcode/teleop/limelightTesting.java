package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.frozenmilk.dairy.core.FeatureRegistrar;

@TeleOp
@Config
public class limelightTesting extends OpMode {

    private static Limelight3A limelight;


    @Override
    public void init() {
        //claw = hardwareMap.get(ServoImplEx.class,"claw");
        //claw.setDirection(Servo.Direction.REVERSE);
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
        Pose current = new Pose(0,0,0);
        boolean red = false;
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
                    red = true;

                    current = new Pose(botpose.getPosition().y*-39.37+xOffset,botpose.getPosition().x*39.37+yOffset,botpose.getOrientation().getYaw(AngleUnit.RADIANS)+headingOffset);
                }
                else{
                    xOffset = 3;
                    yOffset = -3;
                    headingOffset = Math.PI*-0.5;
                    red = false;

                    current = new Pose(botpose.getPosition().y*39.37+xOffset,-1*botpose.getPosition().x*39.37+yOffset,botpose.getOrientation().getYaw(AngleUnit.RADIANS)+headingOffset);

                }

                telemetry.addData("lx: ",botpose.getPosition().x*39.37);
                telemetry.addData("ly: ",botpose.getPosition().y*39.37);
                telemetry.addData("lh: ",botpose.getOrientation().getYaw());
                telemetry.addData("x: ",current.getX());
                telemetry.addData("y: ",current.getY());
                telemetry.addData("h: ",Math.toDegrees(current.getHeading()));
                telemetry.addData("red?: ",red);
                telemetry.update();

            }
        }
    }
    @Override
    public void stop() {

    }

}
