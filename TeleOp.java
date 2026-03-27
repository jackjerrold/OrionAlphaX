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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.teamSelect;

@TeleOp(name = "TeleOp 2026")
public class TeleOp2026 extends OpMode{

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

    private boolean active = true;

    private double rotatedFromIdentity;

    private int BlueGoalID = 0;//TEMPORARY
    private int RedGoalID = 2;//TEMPORARY

    private int goalTagID;
    private int centerTagID = 23;
    private int OppTagID;

    private double driveSpeed = 2;

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

        if (gamepad1.dpadDownWasPressed()){active = !active;}

        if (active){

            turretCam.update();
            sideCam.update();
            turretUpdate();
            rotatedFromIdentity = getRotation();
            telemetry.addData("Rotation", rotatedFromIdentity);

            double x, y, z;

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;

            double rfa = y + x + z;
            double lfa = y - x - z;
            double rba = y - x + z;
            double lba = y + x - z;

            rightFrontMotor.setPower(rfa);
            leftFrontMotor.setPower(lfa);
            rightBackMotor.setPower(rba);
            leftBackMotor.setPower(lba);
        
        }
        else{telemetry.addLine("Press Dpad-Down to activate");}
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
}
