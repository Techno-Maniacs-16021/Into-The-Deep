package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
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


@Autonomous(name="Focal Length Helper", group="Vision")
public class FocalLengthHelper extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {

        int id = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera cam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, id);

        class OneBox extends SampleDistancePipeline {  // reuse colour logic
            @Override public Mat processFrame(Mat input) {
                super.processFrame(input);
                // Print the current bounding-box width each frame
                telemetry.addData("PX width", Double.isNaN(getForwardInch()) ?
                        "none" : (REAL_WIDTH_IN * FOCAL_PX) / getForwardInch());
                telemetry.update();
                return super.processFrame(input);
            }
        }

        cam.setPipeline(new OneBox());
        cam.openCameraDeviceAsync(() -> cam.startStreaming(640,480));

        telemetry.addLine("Point at sample, press ▶");
        telemetry.update();
        waitForStart();

        sleep(10_000);  // look at values, write down the PX width
        cam.stopStreaming(); cam.closeCameraDevice();
    }
}
