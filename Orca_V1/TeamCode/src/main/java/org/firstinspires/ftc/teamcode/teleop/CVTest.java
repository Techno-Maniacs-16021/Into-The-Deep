package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.*;

@Autonomous(name = "Sample-Distance Demo", group = "Vision")
public class SampleDistanceOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private SampleDistancePipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        int id = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera   = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, id);

        pipeline = new SampleDistancePipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(() -> camera.startStreaming(640, 480));

        telemetry.addLine("Init ✔ — press ▶");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Forward (in)",  pipeline.getForwardInch());
            telemetry.addData("Left  <- | ->  Right (in)", pipeline.getLateralInch());
            telemetry.update();
        }

        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
