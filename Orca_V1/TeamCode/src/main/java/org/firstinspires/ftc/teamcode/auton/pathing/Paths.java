package org.firstinspires.ftc.teamcode.auton.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.robots.OrcaV3;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.UnitComponent;

public class Paths {
    public static Map<String, Path> specPathMap = new HashMap<>();
    public static Map<String, Path> samplePathMap = new HashMap<>();

    // ----------------------SPECIMEN PATHS----------------------
    public static Pose startSpec = new Pose(65.5,11,Math.toRadians(180));

    public static Point curve1 = new Point(38.5,5.5);

    public static Point curve2 = new Point(44.5,53);
    public static Point curve3 = new Point(15,35);

    //public static Pose pickup1 = new Pose(17.5,43.5,Math.toRadians(180));
    public static Pose pickup1 = new Pose(15,47,Math.toRadians(180));
    public static Pose pickup2 = new Pose(15,58,Math.toRadians(180));
    public static Pose pickup3 = new Pose(15,64,Math.toRadians(180));

    //public static Pose dropoff1 = new Pose(60,43.5,Math.toRadians(180));
    public static Pose dropoff1 = new Pose(53,47,Math.toRadians(180));
    public static Pose dropoff2 = new Pose(53,58,Math.toRadians(180));
    public static Pose dropoff3 = new Pose(53,64,Math.toRadians(180));

    public static Pose pause = new Pose(58,48,Math.toRadians(180));

    public static Point dropCurve = new Point(38.5,5.5);

    public static Pose specDrop = new Pose(31,5.5,Math.toRadians(180));
    public static Pose specCollect = new Pose(63,48,Math.toRadians(180));
    // ------------------------------------------------------

    // ----------------------SAMPLE PATHS----------------------
    public static Pose startSample = new Pose(65.5,-11,Math.toRadians(180));

    public static Pose sample1 = new Pose(48,-60,Math.toRadians(160));
    public static Pose sample2 = new Pose(48,-60,Math.toRadians(180));
    public static Pose sample3 = new Pose(48,-60,Math.toRadians(200));

    public static Point intakeCurve = new Point(15,-36);

    public static Pose sampleIntake = new Pose(15,-24,Math.toRadians(90));

    public static Pose sampleDrop = new Pose(55,-55,Math.toRadians(135));
    // ------------------------------------------------------

    public static void init() {

        specPathMap.put("firstDeposit-Spec", createPath(
                new BezierLine(
                        new Point(startSpec),
                        new Point(specDrop)),
                //start.getHeading(), --> these are commented out so that it is constant and not linear
                specDrop.getHeading()
        ));

        specPathMap.put("pick1-Spec", createPath(
                new BezierCurve(
                        new Point(specDrop),
                        curve1,
                        curve2,
                        curve3,
                        new Point(pickup1)),
                //specDrop.getHeading(),
                pickup1.getHeading()
        ));

        specPathMap.put("drop1-Spec", createPath(
                new BezierLine(
                        new Point(pickup1),
                        new Point(dropoff1)),
                //pickup1.getHeading(),
                dropoff1.getHeading()
        ));

        specPathMap.put("pick2-Spec", createPath(
                new BezierCurve(
                        new Point(dropoff1),
                        new Point(pickup1),
                        new Point(pickup2)),
                //dropoff1.getHeading(),
                pickup2.getHeading()
        ));

        specPathMap.put("drop2-Spec", createPath(
                new BezierLine(
                        new Point(pickup2),
                        new Point(dropoff2)),
                //pickup2.getHeading(),
                dropoff2.getHeading()
        ));

        specPathMap.put("pick3-Spec", createPath(
                new BezierCurve(
                        new Point(dropoff2),
                        new Point(pickup2),
                        new Point(pickup3)),
                //dropoff2.getHeading(),
                pickup3.getHeading()
        ));

        specPathMap.put("drop3-Spec", createPath(
                new BezierLine(
                        new Point(pickup3),
                        new Point(dropoff3)),
                //pickup3.getHeading(),
                dropoff3.getHeading()
        ));

        specPathMap.put("firstAlign-Spec", createPath(
                new BezierLine(
                        new Point(dropoff3),
                        new Point(pause)),
                //dropoff3.getHeading(),
                pause.getHeading()
        ));

        specPathMap.put("collect-Spec", createPath(
                new BezierLine(
                        new Point(pause),
                        new Point(specCollect)),
                //pause.getHeading(),
                specCollect.getHeading()
        ));

        specPathMap.put("deposit-Spec", createPath(
                new BezierCurve(
                        new Point(specCollect),
                        dropCurve,
                        new Point(specDrop)),
                //specCollect.getHeading(),
                specDrop.getHeading()
        ));
        specPathMap.put("deposit-Spec3", createPath(
                new BezierCurve(
                        new Point(specCollect),
                        dropCurve,
                        new Point(specDrop)),
                //specCollect.getHeading(),
                specDrop.getHeading()
        ));
        specPathMap.put("deposit-Spec4", createPath(
                new BezierCurve(
                        new Point(specCollect),
                        dropCurve,
                        new Point(specDrop)),
                //specCollect.getHeading(),
                specDrop.getHeading()
        ));
        specPathMap.put("deposit-Spec5", createPath(
                new BezierCurve(
                        new Point(specCollect),
                        dropCurve,
                        new Point(specDrop)),
                //specCollect.getHeading(),
                specDrop.getHeading()
        ));
        specPathMap.put("align-Spec", createPath(
                new BezierLine(
                        new Point(specDrop),
                        new Point(pause)),
                //specDrop.getHeading(),
                pause.getHeading()
        ));

        samplePathMap.put("firstDeposit-Sample",createPath(
                new BezierLine(
                        new Point(startSample),
                        new Point(sampleDrop)),
                startSample.getHeading(),
                sampleDrop.getHeading()
        ));

        samplePathMap.put("pick1-Sample",createPath(
                new BezierLine(
                        new Point(sampleDrop),
                        new Point(sample1)),
                sampleDrop.getHeading(),
                sample1.getHeading()
        ));

        samplePathMap.put("drop1-Sample",createPath(
                new BezierLine(
                        new Point(sample1),
                        new Point(sampleDrop)),
                sample1.getHeading(),
                sampleDrop.getHeading()
        ));

        samplePathMap.put("pick2-Sample",createPath(
                new BezierLine(
                        new Point(sampleDrop),
                        new Point(sample2)),
                sampleDrop.getHeading(),
                sample2.getHeading()
        ));

        samplePathMap.put("drop2-Sample",createPath(
                new BezierLine(
                        new Point(sample2),
                        new Point(sampleDrop)),
                sample2.getHeading(),
                sampleDrop.getHeading()
        ));

        samplePathMap.put("pick3-Sample",createPath(
                new BezierLine(
                        new Point(sampleDrop),
                        new Point(sample3)),
                sampleDrop.getHeading(),
                sample3.getHeading()
        ));

        samplePathMap.put("drop3-Sample",createPath(
                new BezierLine(
                        new Point(sample3),
                        new Point(sampleDrop)),
                sample3.getHeading(),
                sampleDrop.getHeading()
        ));

        samplePathMap.put("collect-Sample",createPath(
                new BezierCurve(
                        new Point(sampleDrop),
                        intakeCurve,
                        new Point(sampleIntake)),
                sampleDrop.getHeading(),
                sampleIntake.getHeading()
        ));

        samplePathMap.put("deposit-Sample",createPath(
                new BezierCurve(
                        new Point(sampleIntake),
                        intakeCurve,
                        new Point(sampleDrop)),
                sampleIntake.getHeading(),
                sampleDrop.getHeading()
        ));



    }

    /*public static Path pathTo(BezierLine line, Follower follower) {
        return createPath(
                new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY()),
                        new Point(line.getLastControlPoint().getX(), line.getLastControlPoint().getY())
                )
        );
    }

    public static Path pathTo(BezierCurve curve, Follower follower) {
        ArrayList<Point> controlPoints = curve.getControlPoints();

        // Always include the first point (follower position)
        List<Point> points = new ArrayList<>();
        points.add(new Point(follower.getPose().getX(), follower.getPose().getY()));

        // Add as many control points as available
        points.addAll(controlPoints);

        // Add the last control point
        points.add(curve.getLastControlPoint());

        return createPath(new BezierCurve(points.toArray(new Point[0])));
    }*/

    // Constant Heading (line)
    private static Path createPath(BezierLine line, double heading) {
        Path path = new Path(line);
        path.setConstantHeadingInterpolation(heading);
        return path;
    }

    // Linear Heading (line)
    private static Path createPath(BezierLine line, double startHeading, double endHeading) {
        Path path = new Path(line);
        path.setLinearHeadingInterpolation(startHeading, endHeading);
        return path;
    }

    // Tangent Heading (line)
    private static Path createPath(BezierLine line, boolean tangent) {
        Path path = new Path(line);
        if (tangent) path.setTangentHeadingInterpolation();
        return path;
    }

    // Constant Heading (curve)
    private static Path createPath(BezierCurve curve, double heading) {
        Path path = new Path(curve);
        path.setConstantHeadingInterpolation(heading); // Constant heading
        return path;
    }

    // Linear Heading (curve)
    private static Path createPath(BezierCurve curve, double startHeading, double endHeading) {
        Path path = new Path(curve);
        path.setLinearHeadingInterpolation(startHeading, endHeading); // Linear heading
        return path;
    }

    // Tangent Heading (curve)
    private static Path createPath(BezierCurve curve, boolean tangent) {
        Path path = new Path(curve);
        if (tangent) path.setTangentHeadingInterpolation(); // Tangent heading
        return path;
    }
}