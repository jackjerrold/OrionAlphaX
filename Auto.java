package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.teamSelect;

@Autonomous(name = "AUTO 2026")
public class Auto2026 extends OpMode{

    AprilTagWebcam turretCam, sideCam;
    teamSelect teamSel;

    private Blinker control_Hub;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    private DcMotorEx accelMotor;
    private CRServo turret;

    private IMU imu;
    private AngularVelocity myRobotAngularVelocity;

    private double rotatedFromIdentity;

    private int BlueGoalID = 0;//TEMPORARY
    private int RedGoalID = 2;//TEMPORARY

    private int goalTagID;
    private int centerTagID = 1;//TEMPORARY
    private int OppTagID;

    private double[] blueGoalPos = {2,2};
    private double[] centerPos = {1,2};
    private double[] redGoalPos = {0,2};


    private double driveSpeed = 2;
    private double[] currentPosition = {1,1} //starPos

    private int state;

    @Override
    public void init(){

        teamSel = new teamSelect();

        turretCam = new AprilTagWebcam();
        sideCam = new AprilTagWebcam();
        
        hardwareInit(hardwareMap);
        telemetry.addLine("Hardware Initialised");
        turretCam.init(hardwareMap, telemetry,"Webcam 1");
        sideCam.init(hardwareMap, telemetry, "Webcam 2");
        telemetry.addLine("Cameras Initialised");
        

        telemetry.addLine("Press to start...");
    }

    @Override
    public void loop(){

        currentPosition = UpdatePos()

        switch(state){

            case 0:
                if(MoveTo(0, 0)){
                    state++;
                }
                break;

            case 1:
                if(MoveTo(1, 1)){
                    state++;
                }
                break;

            case 2:
                stopMotors();
                break;
    }
    }

    public void MoveTo(double[] targetPos){
        double currentX = currentPosition[0];
        double currentY = currentPosition[1];

        double errorX = targetX - currentX;
        double errorY = targetY - currentY;

        double tolerance = 0.2;

        if(Math.abs(errorX) < tolerance && Math.abs(errorY) < tolerance){
            stopMotors();
            return true;
        }

        double kP = 0.1;

        double xPower = errorX * kP;
        double yPower = errorY * kP;

        xPower = Math.max(-1, Math.min(1, xPower));
        yPower = Math.max(-1, Math.min(1, yPower));

        double lf = yPower + xPower;
        double rf = yPower - xPower;
        double lb = yPower - xPower;
        double rb = yPower + xPower;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                            Math.max(Math.abs(lb), Math.abs(rb)));
        if(max > 1){
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftBackMotor.setPower(lb);
        rightBackMotor.setPower(rb);

        return false;
    }

    public void UpdatePos(){
        AprilTagDetection centerTag = sideCam.getTagBySpecificId(centerTagID);
        AprilTagDetection redTag = sideCam.getTagBySpecificId(RedGoalID);
        AprilTagDetection blueTag = sideCam.getTagBySpecificId(BlueGoalID);

        if (centerTag == null && goalTag == null && OppTag == null){
            telemetry.addLine("No tag identified");
        }

        ArrayList<Double> positionX = new ArrayList<Double>();
        ArrayList<Double> positionY = new ArrayList<Double>();

        if(centerTagTag != null){
            positionX.add(centerPos[0] - centerTag.ftcPose.x);
            positionY.add(centerPos[1] -  centerTag.ftcPose.y);
        }
        
        
        if(redTag != null){
            positionX.add(redGoalPos[0] - redTag.ftcPose.x);
            positionY.add(redGoalPos[1] redTag.ftcPose.y);
        }

        
        if(blueTag != null){
            positionX.add(blueGoalPos[0] - blueTag.ftcPose.x);
            positionY.add(blueGoalPos[1] - blueTag.ftcPose.y);
        }

        double totalX = 0;
        for (int i = 0; i < positionX.size(); i++){
            totalX += positionX.get(i)
        }

        double totalY = 0;
        for (int i = 0; i < positionY.size(); i++) {
            totalY += positionY.get(i);
        }

        return new double[]{totalX/positionX.size(),totalY/positionY.size()};
    }

    public void TeamIdentify(){
        if (teamSel.GetAlliance().equals("BLUE")){
            goalTagID = BlueGoalID;
            OppTagID = RedGoalID;
        }
        else{
            goalTagID = RedGoalID;
            OppTagID = BlueGoalID;
        }
    }

    public void hardwareInit(HardwareMap hdwr){
        turret = hdwr.get(CRServo.class, "servo1");

        leftFrontMotor = hdwr.get(DcMotor.class, "lf");
        leftBackMotor = hdwr.get(DcMotor.class, "lb");
        rightFrontMotor = hdwr.get(DcMotor.class, "rf");
        rightBackMotor = hdwr.get(DcMotor.class, "rb");

        imu = hardwareMap.get(IMU.class, "IMU");
        IMU.Parameters  parameters;
        parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.resetYaw();

        imu.initialize(parameters);

        myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        
    }
    
    public void turretUpdate(){
        
        AprilTagDetection goalTag = turretCam.getTagBySpecificId(goalTagID);
        
        if(goalTag != null){
            double power = -goalTag.ftcPose.bearing/20;
            turret.setPower(power);
            telemetry.addData("Servo Power", power);
        }
        else{
            turret.setPower(0);
            telemetry.addData("Servo Power", 0);
        }
    }
    
    public double getRotation(){
        AprilTagDetection centerTag = sideCam.getTagBySpecificId(centerTagID);
        AprilTagDetection goalTag = sideCam.getTagBySpecificId(goalTagID);
        AprilTagDetection OppTag = sideCam.getTagBySpecificId(OppTagID);

        if (centerTag == null && goalTag == null && OppTag == null){

            myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            double zRotationRate = myRobotAngularVelocity.zRotationRate;
            return (rotatedFromIdentity + zRotationRate);
        }

        ArrayList<Double> angles = new ArrayList<Double>();

        if(centerTag != null){
            angles.add(centerTag.ftcPose.bearing - 90);
        }
        
        
        if(goalTag != null){
            angles.add(goalTag.ftcPose.bearing + 45);
        }

        
        if(OppTag != null){
            angles.add(OppTag.ftcPose.bearing - 45);
        }

        double totalAngle = 0;
        for (int i = 0; i < angles.size(); i++) {
            totalAngle += angles.get(i);
        }

        return (totalAngle/angles.size());
    }

    public void stopMotors(){
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
