package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auton.pathing.Paths;
import org.firstinspires.ftc.teamcode.robots.OrcaV2;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

@Mercurial.Attach
@BulkRead.Attach
@Autonomous
@Config
public class FiveSpec extends OpMode {

    OrcaV2 orca;

    @Override
    public void init() {
        orca = new OrcaV2(hardwareMap, new Pose2d(0,0,Math.toRadians(0)));
    }

    @Override
    public void loop() {
    }

    @Override
    public void start() {
        new Sequential(
                new Parallel ( //SPECIMEN DROP
                        orca.follow(Paths.pathList.get(0)),
                        new Sequential(
                                //set specimen + smart wait + deposit specimen
                        )
                ),
                new Parallel( //PICKUP 1
                        orca.follow(Paths.pathList.get(1))
                        //intake ground
                        //retract deposit
                ),
                new Parallel( //DROPOFF 1
                        orca.turnTo(Math.toRadians(-40.3)),
                        new Sequential(
                                //smart wait + outtake ground
                        )
                ),
                new Parallel( //PICKUP 2
                        orca.follow(Paths.pathList.get(2))
                        //set intake

                ),
                //intake ground
                new Parallel( //DROPOFF 2
                        orca.turnTo(Math.toRadians(-35)),
                        new Sequential(
                                //smart wait + outtake ground
                        )
                ),
                new Parallel( //PICKUP 3
                        orca.follow(Paths.pathList.get(3))
                        //set intake
                ),
                //intake ground
                new Parallel( //DROP OFF 3 + COLLECT SPECIMEN
                        orca.follow(Paths.pathList.get(4)),
                        orca.turnTo(Math.toRadians(90)),
                        orca.follow(Paths.pathList.get(5)),
                        new Sequential(
                                //set intake
                                //smart wait
                                //outtake ground
                                //retract intake
                                //ready pickup spec
                                //smart wait
                                //pickup spec
                                //smart wait
                        )
                ),
                new Parallel( //SPECIMEN DROP
                        orca.follow(Paths.pathList.get(6)),
                        new Sequential(
                                //set specimen
                                //smart wait
                                //deposit specimen
                        )
                ),
                new Parallel( //COLLECT SPECIMEN
                        orca.follow(Paths.pathList.get(7)),
                        new Sequential(
                                //retract deposit
                                //ready pickup spec
                                //smart wait
                                //pickup spec
                                //smart wait
                        )
                )
        ).schedule();
    }
}