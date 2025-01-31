package org.firstinspires.ftc.teamcode.auton.pathing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Paths {

    public static ArrayList<Path> pathList = new ArrayList<>();

    public static Point start = new Point(11,-65.5,Point.CARTESIAN);

    public static Point pickup1 = new Point(43,-43.4,Point.CARTESIAN);
    public static Point pickup2 = new Point(42.7,-41.5,Point.CARTESIAN);
    public static Point pickup3 = new Point(42.3,-26.9,Point.CARTESIAN);

    //public static Point dropoff1 = new Point(34,-43.4,Point.CARTESIAN);
    //public static Point dropoff2 = new Point(42.7,-41.5,Point.CARTESIAN);
    public static Point dropoff3 = new Point(42.3,-60,Point.CARTESIAN);

    public static Point specDrop = new Point(11,-34.85,Point.CARTESIAN);
    public static Point specCollect = new Point(42.3,-63.8,Point.CARTESIAN);

    public static void init() {
        Collections.addAll(pathList,
                createPath( //specimen drop
                        new BezierLine(
                                start,
                                specDrop)
                ),
                createPath( //pickup 1
                        new BezierLine(
                                specDrop,
                                pickup1),
                        Math.toRadians(270),
                        Math.toRadians(44.5)
                ),
                createPath( //pickup 2
                        new BezierLine(
                                pickup1,
                                pickup2)
                ),
                createPath( //pickup 3
                        new BezierLine(
                                pickup2,
                                pickup3)
                ),
                createPath( //dropoff 3
                        new BezierLine(
                                pickup3,
                                dropoff3)
                ),
                createPath( //specimen collect
                        new BezierLine(
                                dropoff3,
                                specCollect)
                ),
                createPath( //specimen cycle (drop)
                        new BezierLine(
                                specCollect,
                                specDrop)
                ),
                createPath( //specimen cycle (collect)
                        new BezierLine(
                                specDrop,
                                specCollect)
                ),
                createPath( //example curve
                        new BezierCurve(
                                new Point(40.000, 67.000, Point.CARTESIAN),
                                new Point(31.500, 66.000, Point.CARTESIAN),
                                new Point(11.000, 12.000, Point.CARTESIAN),
                                new Point(68.000, 49.500, Point.CARTESIAN),
                                new Point(58.500, 23.000, Point.CARTESIAN)
                        )
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