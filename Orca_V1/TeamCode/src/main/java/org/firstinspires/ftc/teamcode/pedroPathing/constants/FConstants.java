package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {

        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "lf";
        FollowerConstants.leftRearMotorName = "lb";
        FollowerConstants.rightFrontMotorName = "rf";
        FollowerConstants.rightRearMotorName = "rb";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 15.967;

        FollowerConstants.xMovement = 77.19;
        FollowerConstants.yMovement = 51.4004;//avg(31.56,

        FollowerConstants.forwardZeroPowerAcceleration = -30.7821;
        FollowerConstants.lateralZeroPowerAcceleration = -74.0099; //-87.0782, -91.0975, -93.2338, -91.8052, -82.9478, -82.4219

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;//beta
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2.5,0,0.2,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.008,0,0.00001,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 6;
        FollowerConstants.centripetalScaling = 0.00034;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
