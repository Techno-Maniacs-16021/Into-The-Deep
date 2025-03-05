package org.firstinspires.ftc.teamcode.robots;

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
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

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


    // init code might go in here
    public static void teleopInit (){
        follower.startTeleopDrive();
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
                    return INSTANCE.deposit.slidesReachedTarget();
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
    public static Lambda follow(Path path, boolean holdEnd) {
        return new Lambda("follow-path")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(path, holdEnd))
                .setExecute(() -> {
                    follower.update();
                    /*telemetry.addData("x", follower.getPose().getX());
                    telemetry.addData("y", follower.getPose().getY());
                    telemetry.addData("heading", follower.getPose().getHeading());*/
                })
                .setFinish(() -> !follower.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    @NonNull
    public static Lambda follow(Path p1, Path p2, Path p3, Path p4, Path p5, Path p6, boolean holdEnd) {
        PathChain chain = follower.pathBuilder()
                .addPath(p1)
                .addPath(p2)
                .addPath(p3)
                .addPath(p4)
                .addPath(p5)
                .addPath(p6)
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
                })
                .setFinish(() -> !follower.isBusy())
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
