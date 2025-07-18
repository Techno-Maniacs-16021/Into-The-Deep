package org.firstinspires.ftc.teamcode.robots;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auton.imageRec.IMRec;
import org.firstinspires.ftc.teamcode.auton.pathing.Paths;
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
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
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
    public static IMRec pipeline = new IMRec();

    private static Limelight3A limelight;

    public static boolean continueTransfering = false;

    private static boolean teleDrive = true;
    public static boolean driving = true;

    
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

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

    }
    public static void teleopRefresh(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX, boolean gamepad1LeftStickButton, boolean gamepad1RightStickButton){
        double movementScalar = 1;
        double turnScalar = 1;

        if(gamepad1LeftStickButton) movementScalar = 0.5;
        if(gamepad1RightStickButton) turnScalar = 0.5;
        if(!teleDrive){
            if(gamepad1LeftStickX != 0 || gamepad1LeftStickY != 0 || gamepad1RightStickX != 0){

                follower.breakFollowing();
                teleDrive = true;
                driving = false;
            }
        }
        else{
            if(!driving) {
                follower.startTeleopDrive();
                driving = true;
                System.out.println("starting teleop drive");
            }

            follower.setTeleOpMovementVectors(
                    -gamepad1LeftStickY*movementScalar,
                    -gamepad1LeftStickX*movementScalar,
                    -gamepad1RightStickX*turnScalar,
                    true);
            follower.update();
        }

    }
    public static void setFollowerPower(double pwr){
        follower.setMaxPower(pwr);
    }

    public static void startTeleDrive(){
        teleDrive = true;
    }
    public static void stopTeleDrive(){
        teleDrive = false;
    }

    public static void autoInit (Pose startPose, HardwareMap hardwareMap){
        String opModeName = FeatureRegistrar.getActiveOpModeWrapper().getName();
        deposit.setDepositCommand("init");
        intake.setIntakeCommand("standby");
        follower.setStartingPose(startPose);
    }
    public static void startWebacm(){
        int cameraMonitorViewId = FeatureRegistrar.getActiveOpMode().hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", FeatureRegistrar.getActiveOpMode().hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(FeatureRegistrar.getActiveOpMode().hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new IMRec();
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
    public static Lambda autoSpecCollect(boolean holdEnd, double allowedPositionError) {
        stopTeleDrive();
        boolean cancel = false;
        Pose current = follower.getPose();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

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

            System.out.println("x: "+ current.getX() + " y: " + current.getY()+" heading: "+current.getHeading());
        }
        else{
            cancel = true;
            System.out.println("no target--canceled");
        }

        Path limelightPath = new Path(new BezierCurve(
                new Point(current),
                Paths.clearSubCurve,
                new Point(Paths.leadInPause),
                new Point(Paths.pause)
        ));
        limelightPath.setLinearHeadingInterpolation(current.getHeading(),Paths.pause.getHeading());

        Path specCollectPath = new Path(new BezierLine(
                new Point(Paths.pause),
                new Point(Paths.specCollect)
        ));
        specCollectPath.setConstantHeadingInterpolation(Paths.specCollect.getHeading());

        PathChain chain = follower.pathBuilder()
                .addPath(limelightPath)
                .addPath(specCollectPath)
                .build();
        System.out.println("path created");

        Pose currentFinal = current;

        boolean isCanceled = cancel;

        return new Lambda("specimen-teleop")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    //follower.
                    //follower.breakFollowing();
                    if(!isCanceled){
                        follower.setPose(currentFinal);
                        follower.followPath(chain, holdEnd);
                        System.out.println("path scheduled");
                    }

                })
                .setExecute(() -> {
                    if(!isCanceled)
                        follower.update();
                    if((Math.abs(chain.getPath(1).getLastControlPoint().getX()-follower.getPose().getX())<allowedPositionError&&Math.abs(chain.getPath(1).getLastControlPoint().getY()-follower.getPose().getY())<allowedPositionError)){
                        follower.breakFollowing();
                    }
                })
                .setEnd(interrupted -> {
                    if (interrupted) follower.breakFollowing();
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    boolean isFinish = isCanceled || !follower.isBusy();
                        if(isFinish) {
                            driving = false;
                            startTeleDrive();
                        }
                    return isFinish;
                });
    }


    @NonNull
    public static Lambda specimenTeleOp(boolean holdEnd, double allowedPositionError) {
        Pose current = follower.getPose();

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

                System.out.println("x: "+ current.getX() + " y: " + current.getY()+" heading: "+current.getHeading());

            }
        }

        Path limelightPath = new Path(new BezierCurve(
                new Point(current),
                Paths.clearSubCurve,
                new Point(Paths.leadInPause),
                new Point(Paths.pause)
        ));
        limelightPath.setLinearHeadingInterpolation(current.getHeading(),Paths.pause.getHeading());

        Path specCollectPath = new Path(new BezierLine(
                new Point(Paths.pause),
                new Point(Paths.specCollect)
        ));
        specCollectPath.setConstantHeadingInterpolation(Paths.specCollect.getHeading());

        PathChain chain = follower.pathBuilder()
                .addPath(limelightPath)
                .addPath(specCollectPath)
                .build();
        System.out.println("path created");

        Pose current2 = current;

        return new Lambda("specimen-teleop")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    //follower.
                    //follower.breakFollowing();
                    follower.setStartingPose(current2);
                    follower.followPath(chain, holdEnd);
                    System.out.println("path sceduled");
                })
                .setExecute(() -> {
                    follower.update();
                    //if((Math.abs(collect.getLastControlPoint().getX()-follower.getPose().getX())<allowedPositionError&&Math.abs(collect.getLastControlPoint().getY()-follower.getPose().getY())<allowedPositionError)){
                        //follower.breakFollowing();
                    //}
                })
                .setEnd(interrupted -> {
                    if (interrupted) follower.breakFollowing();
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    return !follower.isBusy();
                });
    }

    @NonNull
    public static Lambda printx(String str) {
        return new Lambda("print")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    System.out.println(str);
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
    public static Lambda retractSpecimenDepositFinal() {
        return new Lambda("retract-specimen-deposit-final")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    INSTANCE.deposit.specimenRetractFinal();
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
                    System.out.println("setIntake");

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

        return new Lambda("set-intake-sub-egac")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e

                    double newX =

                            ((pipeline.getMidX()-0.5)*16);
//                    follower.getPose().getX()+0.5;

                    Path align = new Path(
                            new BezierLine(
                                new Point(follower.getPose().getX(),0),
                                new Point(follower.getPose().getX()+newX,0)
                    ));
//                    align.setLinearHeadingInterpolation(follower.getPose().getHeading(),Math.toRadians(90));

                    follower.followPath(align,true);



//
//
//
//                    INSTANCE.intake.verticalIntakeMode();
//                    INSTANCE.intake.enablePID();
//                    INSTANCE.intakeAttemptTimer.reset();
//
//
//                    double slide = 1.5;//pipeline.getMidY()*1+1;
//
//                    INSTANCE.intake.setTarget(slide);


                    /*new Parallel(
                            OrcaV3.setIntake(slide),
                            OrcaV3.follow(align,true,0)
                    ).schedule();*/
                })
                .setExecute(() -> {
                    // do w/e
                    follower.update();
                    System.out.println("setSubIntakeEGAC: " + INSTANCE.intake.slidesReachedTarget()+" "+!follower.isBusy());
                    /*if((Math.abs(follower.getCurrentPath().getLastControlPoint().getX()-follower.getPose().getX())<1&&Math.abs(follower.getCurrentPath().getLastControlPoint().getY()-follower.getPose().getY())<1)){
                        follower.breakFollowing();
                    }*/

                })
                .setEnd(interrupted -> {
                    // do w/e
                })
                .setFinish(() -> {
                    // compute and return if the command is finished
                    boolean isFinished = INSTANCE.intake.slidesReachedTarget()&&!follower.isBusy();
                    if(isFinished){
                        INSTANCE.intake.disablePID();
                    }
                    // compute and return if the command is finished
                    return isFinished;
                    //return true;
                });
    }

    @NonNull
    public static Lambda attemptSubIntakeEGAC() {

        return new Lambda("attempt-intake-sub-egac")
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
    public static Lambda sampleDetected() {
        return new Lambda("sample-detected")
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
                    return !continueTransfering;
                });
    }
    @NonNull
    public static Lambda transferCompleted() {
        return new Lambda("transfer-completed")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    continueTransfering = false;
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
                    if(INSTANCE.deposit.isSampleDetected()){
                        continueTransfering = true;
                    }
                    else{
                        continueTransfering = false;
                    }

                })
                .setExecute(() -> {
                    // do w/e
                    if(INSTANCE.deposit.isSampleDetected()){
                        continueTransfering = true;
                    }
                    else{
                        continueTransfering = false;
                    }

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
    public static Lambda stopStream() {
        return new Lambda("stop-cam")
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
    public static Lambda startStream() {
        return new Lambda("start-cam")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    // do w/e
                    startWebacm();
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
    public static Lambda follow(Path path, boolean holdEnd, double allowedPositionError,double zpam) {
        path.setZeroPowerAccelerationMultiplier(zpam);
        return new Lambda("follow-path-zpam")
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

    @NonNull
    public static Lambda followChain(PathChain chain, boolean holdEnd, double allowedPositionError) {
        return new Lambda("follow-pathchain")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(chain, holdEnd))
                .setExecute(() -> {
                    follower.update();
                    /*telemetry.addData("x", follower.getPose().getX());
                    telemetry.addData("y", follower.getPose().getY());
                    telemetry.addData("heading", follower.getPose().getHeading());*/
                    if((Math.abs(chain.getPath(chain.size()-1).getLastControlPoint().getX()-follower.getPose().getX())<allowedPositionError&&Math.abs(chain.getPath(chain.size()-1).getLastControlPoint().getY()-follower.getPose().getY())<allowedPositionError)){
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
