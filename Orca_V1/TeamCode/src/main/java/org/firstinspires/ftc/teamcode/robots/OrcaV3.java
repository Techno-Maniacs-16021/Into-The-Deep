package org.firstinspires.ftc.teamcode.robots;

import static com.pedropathing.follower.FollowerConstants.automaticHoldEnd;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import static org.threeten.bp.zone.ZoneRulesProvider.refresh;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auton.IMRec;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class OrcaV3 implements Subsystem {
    public static final OrcaV3 INSTANCE = new OrcaV3();

    private OrcaV3() { }

    // the annotation class we use to attach this subsystem
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}
    private Dependency<?> dependency =
            Subsystem.DEFAULT_DEPENDENCY
                    .and(new SingleAnnotation<>(Attach.class));
    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }
    private static DcMotorEx leftFront;
    private static DcMotorEx leftRear;
    private static DcMotorEx rightFront;
    private static DcMotorEx rightRear;
    static DepositV3 deposit;
    static IntakeV3 intake;
    static Follower follower;
    static OpenCvWebcam webcam;
    static IMRec pipeline;

    private static Limelight3A limelight;

    
    ElapsedTime intakeAttemptTimer = new ElapsedTime();



    // init code might go in here
    public static void teleopInit (){
        follower.startTeleopDrive();
        //limelight stuff
        limelight = FeatureRegistrar.getActiveOpMode().hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        //deposit.setDepositCommand("specimen");
        leftFront = FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void teleopRefresh(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX){
        follower.setTeleOpMovementVectors(-gamepad1LeftStickY, -gamepad1LeftStickX, -gamepad1RightStickX,true);
        follower.update();
    }
    public static void autoInit (Pose startPose, HardwareMap hardwareMap){
        String opModeName = FeatureRegistrar.getActiveOpModeWrapper().getName();
        deposit.setDepositCommand("init");
        intake.setIntakeCommand("standby");
        follower.setStartingPose(startPose);
    }

    public static Follower follower() {
        return follower;
    }

    public static IntakeV3 intake(){
        return intake;
    }


    public static DepositV3 deposit(){
        return deposit;
    }

    @NonNull
    public static Lambda specimenTeleOp(Pose pause, Path deposit, boolean holdEnd, double allowedPositionError) {
        Pose current = follower.getPose();

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                current = new Pose(botpose.getPosition().x,botpose.getPosition().y,botpose.getOrientation().getYaw());
            }
        }
        Path path = new Path(new BezierLine(current,pause));
        path.setLinearHeadingInterpolation(current.getHeading(),pause.getHeading());
        return new Lambda("specimen-teleop")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    new Parallel(
                            INSTANCE.follow(path,deposit, holdEnd, allowedPositionError),
                            INSTANCE.retractSpecimenDeposit()
                    ).schedule();
                })
                .setExecute(() -> {
                    // do w/e
                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return true;
                });
    }

    @NonNull
    public static Lambda setSpecimen() {
        return new Lambda("set-specimen")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.deposit.depositSpecimen();
                })
                .setExecute(() -> {
                    // do w/e
                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return INSTANCE.deposit.isStateComplete();
                });
    }

    @NonNull
    public static Lambda setSample() {
        return new Lambda("set-sample")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    //xxINSTANCE.deposit.resetGrabTimer();
                    INSTANCE.deposit.setSample();

                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return INSTANCE.deposit.slidesReachedTarget()&&INSTANCE.deposit.isStateComplete();
                });
    }

    @NonNull
    public static Lambda setSample(boolean first) {
        return new Lambda("set-sample-first")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    //xxINSTANCE.deposit.resetGrabTimer();
                    INSTANCE.deposit.setSampleAuto();

                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return INSTANCE.deposit.slidesReachedTarget()&&INSTANCE.deposit.isStateComplete();
                });
    }

    @NonNull
    public static Lambda closeClaw() {
        return new Lambda("close-claw")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.deposit.closeClaw();
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return true;
                });
    }
    @NonNull
    public static Lambda releaseClaw() {
        return new Lambda("release-claw")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.deposit.releaseClaw();
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return true;
                });
    }
    @NonNull
    public static Lambda retractDeposit() {
        return new Lambda("retract-deposit")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.deposit.retract();
                })
                .setExecute(() -> {
                    // do w/e
                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return INSTANCE.deposit.isStateComplete()&&INSTANCE.deposit.slidesReachedTarget();
                });
    }

    @NonNull
    public static Lambda retractSpecimenDeposit() {
        return new Lambda("retract-specimen-deposit")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.deposit.specimenRetract();
                })
                .setExecute(() -> {
                    // do w/e
                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return INSTANCE.deposit.isStateComplete()&&INSTANCE.deposit.slidesReachedTarget();
                });
    }

    @NonNull
    public static Lambda setIntake(double target) {
        return new Lambda("set-intake")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.intake.startIntaking();
                    INSTANCE.intake.enablePID();
                    INSTANCE.intake.setTarget(target);
                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    boolean isFinished = INSTANCE.intake.slidesReachedTarget();
                    if(isFinished){
                        INSTANCE.intake.disablePID();
                    }
                    // compute and return if the command is finished
                    return isFinished;
                });
    }

    @NonNull
    public static Lambda attemptIntake(double slidePower) {
        return new Lambda("attempt-intake")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.intake.startIntaking();
                    INSTANCE.intake.angledIntakeMode();
                    INSTANCE.intake.enableIntakeMotor();
                    INSTANCE.intake.disablePID();
                    INSTANCE.intake.setSlidesPower = slidePower;
                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    boolean isFinished =
                            (!INSTANCE.intake.getSampleColor().equals(INSTANCE.intake.colorToEject)
                            &&!INSTANCE.intake.getSampleColor().equals("none"));
                            //||INSTANCE.intake.slidesReachedTarget();
                    if(isFinished){

                    }
                    // compute and return if the command is finished
                    return isFinished;
                });
    }

    @NonNull
    public static Lambda attemptSubIntake() {

        return new Lambda("attempt-intake-sub")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.intake.startIntaking();
                    INSTANCE.intake.verticalIntakeMode();
                    INSTANCE.intake.enableIntakeMotor();
                    INSTANCE.intake.disablePID();
                    INSTANCE.intakeAttemptTimer.reset();

                })
                .setExecute(() -> {
                    // do w/e
                    if(INSTANCE.intakeAttemptTimer.milliseconds()<1000){
                        INSTANCE.intake.enableIntakeMotor();
                    }
                    else if(INSTANCE.intakeAttemptTimer.milliseconds()<1500){
                        INSTANCE.intake.disableIntakeMotor();
                        INSTANCE.intake.setSlidePower(0.75);
                    }
                    else{
                        INSTANCE.intakeAttemptTimer.reset();
                        INSTANCE.intake.setSlidePower(0);
                    }

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    boolean isFinished =
                            (!INSTANCE.intake.getSampleColor().equals(INSTANCE.intake.colorToEject)
                            &&!INSTANCE.intake.getSampleColor().equals("none"))
                            ||INSTANCE.intake.slidesReachedTarget();
                    if(isFinished){
                        INSTANCE.intake.enableIntakeMotor();
                    }
                    // compute and return if the command is finished
                    return isFinished;
                });
    }

    @NonNull
    public static Lambda setSubIntakeEGAC() {

        return new Lambda("attempt-intake-sub")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    double newX = follower.getPose().getX()+(0.5-pipeline.getMidX()*5);
                    double slide = pipeline.getMidY()*1+1;

                    Path align = new Path(
                            new BezierLine(
                                new Point(follower.getPose().getX(),follower.getPose().getY()),
                                new Point(newX,follower.getPose().getY())
                    ));
                    align.setConstantHeadingInterpolation(follower.getPose().getHeading());

                    INSTANCE.intake.startIntaking();
                    INSTANCE.intake.enablePID();
                    INSTANCE.intakeAttemptTimer.reset();

                    new Parallel(
                            follow(align,true,0),
                            setIntake(slide)
                    ).schedule();




                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return true;
                });
    }

    @NonNull
    public static Lambda attemptSubIntakeEGAC() {

        return new Lambda("attempt-intake-sub")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.intake.startIntaking();
                    INSTANCE.intake.verticalIntakeMode();
                    INSTANCE.intake.enableIntakeMotor();
                    INSTANCE.intakeAttemptTimer.reset();

                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    boolean isFinished =
                            (!INSTANCE.intake.getSampleColor().equals(INSTANCE.intake.colorToEject)
                                    &&!INSTANCE.intake.getSampleColor().equals("none"))
                                    ||INSTANCE.intakeAttemptTimer.milliseconds()>3000;
                    if(isFinished){
                        INSTANCE.intake.enableIntakeMotor();
                        INSTANCE.intake.disablePID();

                    }
                    // compute and return if the command is finished
                    return isFinished;
                });
    }

    @NonNull
    public static Lambda retractIntake() {
        return new Lambda("attempt-intake")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.intake.setSlidePower(0);
                    INSTANCE.intake.disableIntakeMotor();
                    INSTANCE.intake.verticalIntakeMode();
                    INSTANCE.intake.retract();
                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return true;
                });
    }

    @NonNull
    public static Lambda waitForTransfer() {
        return new Lambda("wait-for-transfer")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return INSTANCE.deposit.checkCommand("depositSample")&&INSTANCE.deposit.isStateComplete();
                });
    }

    @NonNull
    public static Lambda pauseViewPort() {
        return new Lambda("pause-cam")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    webcam.pauseViewport();
                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return true;
                });
    }

    @NonNull
    public static Lambda resumeViewPort() {
        return new Lambda("pause-cam")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    webcam.resumeViewport();
                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return true;
                });
    }

    @NonNull
    public static Lambda stopStreaming() {
        return new Lambda("pause-cam")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    webcam.stopStreaming();
                })
                .setExecute(() -> {
                    // do w/e

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return true;
                });
    }

    @NonNull
    public static Lambda follow(Path path, boolean holdEnd, double allowedPositionError) {
        return new Lambda("follow-path")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(path, holdEnd))
                .setExecute(() -> {
                    follower.update();
                    /*telemetry.addData("x", follower.getPose().getX());
                    telemetry.addData("y", follower.getPose().getY());
                    telemetry.addData("heading", follower.getPose().getHeading());*/
                    if((Math.abs(path.getLastControlPoint().getX()-follower.getPose().getX())<allowedPositionError&&Math.abs(path.getLastControlPoint().getY()-follower.getPose().getY())<allowedPositionError)){
                        follower.breakFollowing();
                    }
                })
                .setFinish(() -> (!follower.isBusy()))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }


    @NonNull
    public static Lambda follow(Path p1, Path p2, boolean holdEnd, double allowedPositionError) {
        PathChain chain = follower.pathBuilder()
                .addPath(p1)
                .addPath(p2)
                .build();
        return new Lambda("follow-pathchain")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(chain, holdEnd))
                .setExecute(() -> {
                    follower.update();
                    /*telemetry.addData("x", follower.getPose().getX());
                    telemetry.addData("y", follower.getPose().getY());
                    telemetry.addData("heading", follower.getPose().getHeading());*/
                    if((Math.abs(p2.getLastControlPoint().getX()-follower.getPose().getX())<allowedPositionError&&Math.abs(p2.getLastControlPoint().getY()-follower.getPose().getY())<allowedPositionError)){
                        follower.breakFollowing();
                    }
                })
                .setFinish(() -> (!follower.isBusy()))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    public static Lambda follow(Path p1, Path p2, Path p3, boolean holdEnd, double allowedPositionError) {
        PathChain chain = follower.pathBuilder()
                .addPath(p1)
                .addPath(p2)
                .addPath(p3)
                .build();
        return new Lambda("follow-pathchain-3")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(chain, holdEnd))
                .setExecute(() -> {
                    follower.update();
                    /*telemetry.addData("x", follower.getPose().getX());
                    telemetry.addData("y", follower.getPose().getY());
                    telemetry.addData("heading", follower.getPose().getHeading());*/
                    if((Math.abs(p3.getLastControlPoint().getX()-follower.getPose().getX())<allowedPositionError&&Math.abs(p3.getLastControlPoint().getY()-follower.getPose().getY())<allowedPositionError)){
                        follower.breakFollowing();
                    }
                })
                .setFinish(() -> (!follower.isBusy()))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    @NonNull
    public static Lambda follow(Path p1, Path p2, double zpam, boolean holdEnd, double allowedPositionError) {
        PathChain chain = follower.pathBuilder()
                .addPath(p1)
                .addPath(p2)
                .setZeroPowerAccelerationMultiplier(zpam)
                .build();
        return new Lambda("follow-pathchain-zpam")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(chain, holdEnd))
                .setExecute(() -> {
                    follower.update();
                    /*telemetry.addData("x", follower.getPose().getX());
                    telemetry.addData("y", follower.getPose().getY());
                    telemetry.addData("heading", follower.getPose().getHeading());*/
                    if((Math.abs(p2.getLastControlPoint().getX()-follower.getPose().getX())<allowedPositionError&&Math.abs(p2.getLastControlPoint().getY()-follower.getPose().getY())<allowedPositionError)){
                        follower().breakFollowing();
                    }
                })
                .setFinish(() -> (!follower.isBusy()))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }


        @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        // default command should be set up here, not in the constructor


        deposit = new DepositV3(FeatureRegistrar.getActiveOpMode().hardwareMap);
        intake = new IntakeV3(FeatureRegistrar.getActiveOpMode().hardwareMap);
        //follower.setStartingPose(startPose);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(FeatureRegistrar.getActiveOpMode().hardwareMap);

            int cameraMonitorViewId = FeatureRegistrar.getActiveOpMode().hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", FeatureRegistrar.getActiveOpMode().hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(FeatureRegistrar.getActiveOpMode().hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            IMRec pipeline = new IMRec();
            webcam.setPipeline(pipeline);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
            webcam.pauseViewport();
    }
    // or here
    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {}

    // and you might put periodic code in these
    @Override
    public void preUserInitLoopHook(@NonNull Wrapper opMode) {}
    @Override
    public void postUserInitLoopHook(@NonNull Wrapper opMode) {
        intake.refresh();
        deposit.refresh();

    }
    @Override
    public void preUserLoopHook(@NonNull Wrapper opMode) {}
    // or these
    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
        intake.refresh();
        deposit.refresh();
    }

    // and stopping code can go in here
    @Override
    public void preUserStopHook(@NonNull Wrapper opMode) {}
    // or here
    @Override
    public void postUserStopHook(@NonNull Wrapper opMode) {}

    // see the feature dev notes on when to use cleanup vs postStop
    @Override
    public void cleanup(@NonNull Wrapper opMode) {}


}
