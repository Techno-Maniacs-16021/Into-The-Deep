package org.firstinspires.ftc.teamcode.auton.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.UnitComponent;

public class Paths {

    public static ArrayList<Path> pathList = new ArrayList<>();

    public static Pose start = new Pose(63.5,11,Math.toRadians(0));

    public static Point curve1 = new Point(48,48);
    public static Point curve2 = new Point(15,35);

    public static Pose pickup1 = new Pose(17.5,43.5,Math.toRadians(0));
    public static Pose pickup2 = new Pose(17.5,55.5,Math.toRadians(0));
    public static Pose pickup3 = new Pose(17.5,63.5,Math.toRadians(0));

    public static Pose dropoff1 = new Pose(60,43.5,Math.toRadians(0));
    public static Pose dropoff2 = new Pose(60,55.5,Math.toRadians(0));
    public static Pose dropoff3 = new Pose(60,63.5,Math.toRadians(0));

    public static Pose pause = new Pose(45,42.3,Math.toRadians(180));

    public static Pose specDrop = new Pose(38,11,Math.toRadians(0));
    public static Pose specCollect = new Pose(56,42.3,Math.toRadians(180));

    public static void init() {
        Collections.addAll(pathList,
                createPath(//specimen drop
                        new BezierLine(
                                new Point(start),
                                new Point(specDrop))
                ),
                createPath( //pickup 1
                        new BezierCurve(
                                new Point(specDrop),
                                curve1,
                                curve2,
                                new Point(pickup1))
                ),
                createPath( //dropoff 1
                        new BezierLine(
                                new Point(pickup1),
                                new Point(dropoff1))
                ),
                createPath( //pickup 2
                        new BezierCurve(
                                new Point(dropoff1),
                                new Point(pickup1),
                                new Point(pickup2))
                ),
                createPath( //dropoff 2
                        new BezierLine(
                                new Point(pickup2),
                                new Point(dropoff2))
                ),
                createPath( //5
                        new BezierLine(
                                new Point(dropoff2),
                                new Point(pause)),
                        dropoff2.getHeading(),
                        pause.getHeading()
                        ),
                createPath(//6
                        new BezierLine(
                                new Point(pause),
                                new Point(specCollect)),
                        pause.getHeading()
                ),
                createPath( //junk file - 7
                        new BezierLine(
                              new Point(specCollect),
                              new Point(pause))
                ),
                createPath( //specimen cycle (drop) - 8
                        new BezierLine(
                                new Point(specCollect),
                                new Point(specDrop)),
                        specCollect.getHeading(),
                        specDrop.getHeading()
                ),
                createPath( //specimen cycle (collect) - 9
                        new BezierLine(
                                new Point(specDrop),
                                new Point(pause)
                        ),
                                specDrop.getHeading(),
                                pause.getHeading()
                )
        );

    }

    public static Path pathTo(BezierLine line, Follower follower) {
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
    }


    private static Path createPath(BezierLine line, double heading) {
        Path path = new Path(line);
        path.setConstantHeadingInterpolation(heading); // Constant heading
        return path;
    }

    private static Path createPath(BezierLine line, double startHeading, double endHeading) {
        Path path = new Path(line);
        path.setLinearHeadingInterpolation(startHeading, endHeading); // Linear heading
        return path;
    }

    private static Path createPath(BezierLine line, boolean tangent) {
        Path path = new Path(line);
        if (tangent) path.setTangentHeadingInterpolation();
        return path;
    }

    private static Path createPath(BezierLine line) {
        Path path = new Path(line);
        path.setConstantHeadingInterpolation(0);
        return path;
    }

    private static Path createPath(BezierCurve curve) {
        Path path = new Path(curve);
        path.setConstantHeadingInterpolation(0);
        return path;
    }

    // Constant Heading
    private static Path createPath(BezierCurve curve, double heading) {
        Path path = new Path(curve);
        path.setConstantHeadingInterpolation(heading); // Constant heading
        return path;
    }

    // Linear Heading
    private static Path createPath(BezierCurve curve, double startHeading, double endHeading) {
        Path path = new Path(curve);
        path.setLinearHeadingInterpolation(startHeading, endHeading); // Linear heading
        return path;
    }

    // Tangent or Default Constant Heading
    private static Path createPath(BezierCurve curve, boolean tangent) {
        Path path = new Path(curve);
        if (tangent) path.setTangentHeadingInterpolation(); // Tangent heading
        else path.setConstantHeadingInterpolation(0);      // Default constant heading
        return path;
    }
}