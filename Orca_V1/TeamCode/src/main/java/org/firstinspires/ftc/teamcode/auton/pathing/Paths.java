package org.firstinspires.ftc.teamcode.auton.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Paths {
    public static Map<String, Path> pathMap = new HashMap<>();

    public static Pose start = new Pose(63.5,11,Math.toRadians(180));

    public static Point curve1 = new Point(48,48);
    public static Point curve2 = new Point(15,35);

    public static Pose pickup1 = new Pose(17.5,43.5,Math.toRadians(180));
    public static Pose pickup2 = new Pose(17.5,55.5,Math.toRadians(180));
    public static Pose pickup3 = new Pose(17.5,63.5,Math.toRadians(180));

    public static Pose dropoff1 = new Pose(60,43.5,Math.toRadians(180));
    public static Pose dropoff2 = new Pose(60,55.5,Math.toRadians(180));
    public static Pose dropoff3 = new Pose(60,63.5,Math.toRadians(180));

    public static Pose pause = new Pose(45,42.3,Math.toRadians(180));

    public static Pose specDrop = new Pose(38,11,Math.toRadians(180));
    public static Pose specCollect = new Pose(56,42.3,Math.toRadians(180));

    public static void init() {
        pathMap.put("firstDeposit", createPath(
                new BezierLine(
                        new Point(start),
                        new Point(specDrop)),
                //start.getHeading(), --> these are commented out so that it is constant and not linear
                specDrop.getHeading()
        ));

        pathMap.put("pick1", createPath(
                new BezierCurve(
                        new Point(specDrop),
                        curve1,
                        curve2,
                        new Point(pickup1)),
                //specDrop.getHeading(),
                pickup1.getHeading()
        ));

        pathMap.put("drop1", createPath(
                new BezierLine(
                        new Point(pickup1),
                        new Point(dropoff1)),
                //pickup1.getHeading(),
                dropoff1.getHeading()
        ));

        pathMap.put("pick2", createPath(
                new BezierCurve(
                        new Point(dropoff1),
                        new Point(pickup1),
                        new Point(pickup2)),
                //dropoff1.getHeading(),
                pickup2.getHeading()
        ));

        pathMap.put("drop2", createPath(
                new BezierLine(
                        new Point(pickup2),
                        new Point(dropoff2)),
                //pickup2.getHeading(),
                dropoff2.getHeading()
        ));

        pathMap.put("pick3", createPath(
                new BezierCurve(
                        new Point(dropoff2),
                        new Point(pickup2),
                        new Point(pickup3)),
                //dropoff2.getHeading(),
                pickup3.getHeading()
        ));

        pathMap.put("drop3", createPath(
                new BezierLine(
                        new Point(pickup3),
                        new Point(dropoff3)),
                //pickup3.getHeading(),
                dropoff3.getHeading()
        ));

        pathMap.put("firstAlign", createPath(
                new BezierLine(
                        new Point(dropoff3),
                        new Point(pause)),
                //dropoff3.getHeading(),
                pause.getHeading()
        ));

        pathMap.put("collectSpec", createPath(
                new BezierLine(
                        new Point(pause),
                        new Point(specCollect)),
                //pause.getHeading(),
                specCollect.getHeading()
        ));

        pathMap.put("depositSpec", createPath(
                new BezierLine(
                        new Point(specCollect),
                        new Point(specDrop)),
                //specCollect.getHeading(),
                specDrop.getHeading()
        ));

        pathMap.put("alignSpec", createPath(
                new BezierLine(
                        new Point(specDrop),
                        new Point(pause)),
                //specDrop.getHeading(),
                pause.getHeading()
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