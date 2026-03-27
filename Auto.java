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

    double lastLF = 0, lastRF = 0, lastLB = 0, lastRB = 0;

    double encoderX = 1; // start pos
    double encoderY = 1;

    double TICKS_TO_METERS = 0.0005;

    private DcMotorEx accelMotor;
    private CRServo turret;

    private IMU imu;
    private AngularVelocity myRobotAngularVelocity;

    private double rotatedFromIdentity;

    private int BlueGoalID = 0;//TEMPORARY
    private int RedGoalID = 2;//TEMPORARY

    private int goalTagID;
    private int centerTagID = 23;
    private int OppTagID;

    private double[] blueGoalPos = {2,2};
    private double[] centerPos = {1,2};
    private double[] redGoalPos = {0,2};


    private double driveSpeed = 2;
    private double[] currentPosition = {1,1}; //starPos
    
    private double[] p1 = {0,0};

    private int state = 0;

    @Override
    public void init(){

        teamSel = new teamSelect();
        teamSel.TeamIdentify();

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

        turretUpdate();

        currentPosition = UpdatePos();

        switch(state){

            case 0:
                if(MoveTo(p1)){
                    state++;
                }
                break;

            case 1:
                stopMotors();
                break;
        }
    }

    public boolean MoveTo(double[] targetPos){

        double currentX = currentPosition[0];
        double currentY = currentPosition[1];

        double errorX = targetPos[0] - currentX;
        double errorY = targetPos[1] - currentY;

        telemetry.addData("errorX", errorX);
        telemetry.addData("errorY", errorY);

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

    public double[] UpdatePos(){

        updateEncoders();

        AprilTagDetection centerTag = sideCam.getTagBySpecificId(centerTagID);
        AprilTagDetection redTag = sideCam.getTagBySpecificId(RedGoalID);
        AprilTagDetection blueTag = sideCam.getTagBySpecificId(BlueGoalID);

        ArrayList<Double> positionX = new ArrayList<>();
        ArrayList<Double> positionY = new ArrayList<>();

        if(centerTag != null){
            positionX.add(centerPos[0] - centerTag.ftcPose.x);
            positionY.add(centerPos[1] - centerTag.ftcPose.y);
        }

        if(redTag != null){
            positionX.add(redGoalPos[0] - redTag.ftcPose.x);
            positionY.add(redGoalPos[1] - redTag.ftcPose.y);
        }

        if(blueTag != null){
            positionX.add(blueGoalPos[0] - blueTag.ftcPose.x);
            positionY.add(blueGoalPos[1] - blueTag.ftcPose.y);
        }

        if(positionX.size() == 0){
            telemetry.addLine("Using Encoders Only");
            return new double[]{encoderX, encoderY};
        }

        double totalX = 0, totalY = 0;

        for(double x : positionX) totalX += x;
        for(double y : positionY) totalY += y;

        double tagX = totalX / positionX.size();
        double tagY = totalY / positionY.size();

        double alpha = 0.7; // trust tags more

        encoderX = alpha * tagX + (1 - alpha) * encoderX;
        encoderY = alpha * tagY + (1 - alpha) * encoderY;

        telemetry.addLine("Using Tag Fusion");

        return new double[]{encoderX, encoderY};
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

    public void updateEncoders(){

        double lf = leftFrontMotor.getCurrentPosition();
        double rf = rightFrontMotor.getCurrentPosition();
        double lb = leftBackMotor.getCurrentPosition();
        double rb = rightBackMotor.getCurrentPosition();

        double dlf = lf - lastLF;
        double drf = rf - lastRF;
        double dlb = lb - lastLB;
        double drb = rb - lastRB;

        lastLF = lf;
        lastRF = rf;
        lastLB = lb;
        lastRB = rb;

        // Mecanum motion
        double dx = (dlf - drf - dlb + drb) / 4.0;
        double dy = (dlf + drf + dlb + drb) / 4.0;

        dx *= TICKS_TO_METERS;
        dy *= TICKS_TO_METERS;

        // Convert to field-centric
        double heading = Math.toRadians(getRotation());

        double fieldX = dx * Math.cos(heading) - dy * Math.sin(heading);
        double fieldY = dx * Math.sin(heading) + dy * Math.cos(heading);

        encoderX += fieldX;
        encoderY += fieldY;
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
