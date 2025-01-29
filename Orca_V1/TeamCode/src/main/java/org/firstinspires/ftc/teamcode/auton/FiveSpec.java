package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
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


    @Override
    public void init() {
        //initialization
    }

    @Override
    public void loop() {
    }

    @Override
    public void start() {
        //auton code goes here
        new Sequential(
                new Parallel(
                     //OrcaV2.followPath(Paths.pathList.get(0));
                )
        ).schedule();
    }
}