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

        FollowerConstants.mass = 15.195;

        //FollowerConstants.xMovement = 77.19;
        //77.1659 75.2182 77.6393
        FollowerConstants.xMovement = 72.044; //77.4026;
        //50.5361 52.1748 51.3676 51.6833
        //FollowerConstants.yMovement = 51.4004;//avg(31.56,
        FollowerConstants.yMovement = 50.8023; //51.0752;
        //FollowerConstants.forwardZeroPowerAcceleration = -30.7821;
        //-33.0362 -53.43 -35.3665 -52.7933 -35.36464 -46.8289 -30.357 -39.7771 -32.1053 -34.04 -33.29 -41.5671
        FollowerConstants.forwardZeroPowerAcceleration = -50.7931;
        //-88.0295 -98.7099 -93.094 -88.6734 -99.5168 -91.2524 -91.2017 , -91.2946
        //FollowerConstants.lateralZeroPowerAcceleration = -74.0099; //-87.0782, -91.0975, -93.2338, -91.8052, -82.9478, -82.4219
        FollowerConstants.lateralZeroPowerAcceleration = -89.85097;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.125,0,0.004,0);
        FollowerConstants.useSecondaryTranslationalPID = false;//beta
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.08,0,0.0025,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.15,0,0.055,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2.5,0,0.2,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.012,0,0.00000012,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.0012,0,0.000000012,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 2.5;
        FollowerConstants.centripetalScaling = 0.00034;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
