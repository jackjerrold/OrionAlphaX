package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(5); //Measure


    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .xVelocity(velocity) //Velocity Tuning
        .yVelocity(velocity)
        .rightFrontMotorName("frontRight")
        .rightRearMotorName("backRight")
        .leftRearMotorName("backLeft")
        .leftFrontMotorName("frontLeft")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(
        0.99, // tValueConstraint: How much of the path must be finished (0.99 = 99%)
        0.1, // velocityConstraint: Must be moving slower than this to finish (inches/sec)
        0.1,// translationalConstraint: How close it must be to the (x,y) target (inches)
        0.01,// headingConstraint: How close the angle must be (radians)
        50// timeoutConstraint: Max time to "correct" at the end (milliseconds)
    );


public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0) //Offsets for odo pods
            .strafePodX(0) //Offsets for odo pods (Tuning)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWING_ARM)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            
        
            
public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .build();
}
    }
}
