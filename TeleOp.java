package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

import java.util.ArrayList;

@Autonomous(name = "AUTO 2026")
public class Auto2026 extends OpMode {

    AprilTagWebcam turretCam, sideCam;

    private DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    private CRServo turret;
    private IMU imu;

    private int BlueGoalID = 0, RedGoalID = 2, centerTagID = 1;
    private int goalTagID, OppTagID;

    private double[] blueGoalPos = {2, 2}, centerPos = {1, 2}, redGoalPos = {0, 2};
    private double[] currentPosition = {1, 1}; 

    private int state = 0;

    @Override
    public void init() {
        turretCam = new AprilTagWebcam();
        sideCam = new AprilTagWebcam();

        hardwareInit(hardwareMap);
        
        turretCam.init(hardwareMap, telemetry, "Webcam 1");
        sideCam.init(hardwareMap, telemetry, "Webcam 2");
        
        // Default alliance setup (should be replaced by your teamSel logic)
        goalTagID = BlueGoalID; 
        OppTagID = RedGoalID;

        telemetry.addLine("Ready to Start");
    }

    @Override
    public void loop() {
        currentPosition = UpdatePos();

        switch (state) {
            case 0:
                if (MoveTo(0.0, 0.0)) state++;
                break;
            case 1:
                if (MoveTo(1.0, 1.0)) state++;
                break;
            case 2:
                stopMotors();
                telemetry.addLine("Path Complete");
                break;
        }
        
        turretUpdate();
        telemetry.addData("X", "%.2f", currentPosition[0]);
        telemetry.addData("Y", "%.2f", currentPosition[1]);
        telemetry.update();
    }

    // Overloaded to handle MoveTo(x, y)
    public boolean MoveTo(double targetX, double targetY) {
        double errorX = targetX - currentPosition[0];
        double errorY = targetY - currentPosition[1];
        double tolerance = 0.2;

        if (Math.abs(errorX) < tolerance && Math.abs(errorY) < tolerance) {
            stopMotors();
            return true;
        }

        double kP = 0.5; // Increased kP slightly for response
        double xPower = Math.max(-1, Math.min(1, errorX * kP));
        double yPower = Math.max(-1, Math.min(1, errorY * kP));

        // Mecanum kinematics
        double lf = yPower + xPower;
        double rf = yPower - xPower;
        double lb = yPower - xPower;
        double rb = yPower + xPower;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max; rf /= max; lb /= max; rb /= max;
        }

        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftBackMotor.setPower(lb);
        rightBackMotor.setPower(rb);

        return false;
    }

    public double[] UpdatePos() {
        AprilTagDetection centerTag = sideCam.getTagBySpecificId(centerTagID);
        AprilTagDetection redTag = sideCam.getTagBySpecificId(RedGoalID);
        AprilTagDetection blueTag = sideCam.getTagBySpecificId(BlueGoalID);

        ArrayList<Double> positionX = new ArrayList<>();
        ArrayList<Double> positionY = new ArrayList<>();

        if (centerTag != null) {
            positionX.add(centerPos[0] - centerTag.ftcPose.x);
            positionY.add(centerPos[1] - centerTag.ftcPose.y);
        }
        if (redTag != null) {
            positionX.add(redGoalPos[0] - redTag.ftcPose.x);
            positionY.add(redGoalPos[1] - redTag.ftcPose.y);
        }
        if (blueTag != null) {
            positionX.add(blueGoalPos[0] - blueTag.ftcPose.x);
            positionY.add(blueGoalPos[1] - blueTag.ftcPose.y);
        }

        if (positionX.isEmpty()) return currentPosition; // Keep last known pos if no tags

        double totalX = 0, totalY = 0;
        for (Double x : positionX) totalX += x;
        for (Double y : positionY) totalY += y;

        return new double[]{totalX / positionX.size(), totalY / positionY.size()};
    }

    public void hardwareInit(HardwareMap hdwr) {
        turret = hdwr.get(CRServo.class, "turret");
        leftFrontMotor = hdwr.get(DcMotor.class, "frontLeft");
        leftBackMotor = hdwr.get(DcMotor.class, "backLeft");
        rightFrontMotor = hdwr.get(DcMotor.class, "frontRight");
        rightBackMotor = hdwr.get(DcMotor.class, "backRight");

        imu = hdwr.get(IMU.class, "IMU");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void turretUpdate() {
        AprilTagDetection goalTag = turretCam.getTagBySpecificId(goalTagID);
        if (goalTag != null) {
            double power = -goalTag.ftcPose.bearing / 20.0;
            turret.setPower(Math.max(-1, Math.min(1, power)));
        } else {
            turret.setPower(0);
        }
    }

    public void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
