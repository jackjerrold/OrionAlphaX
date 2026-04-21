package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@TeleOp(name = "TeleOp 2026")
public class TeleOp2026 extends OpMode{

    AprilTagWebcam turretCam, sideCam;

    private Blinker control_Hub;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    private DcMotorEx intakeMotor;
    private DcMotorEx flywheelMotor;

    private DcMotorEx accelMotor;
    private DcMotorEx turret;//chage to DCMotorEx

    private double BackPlateAngle;//ADD

    //private IMU imu;
    //private AngularVelocity myRobotAngularVelocity;

    private boolean active = true;

    //private double rotatedFromIdentity;

    private int goalTagID = 0;
    private int centerTagID = 1;
    private int OppTagID = 2;

    private double driveSpeed = 2;

    int MIN_POS = -500;
    int MAX_POS = 500;


    @Override
    public void init(){

        turretCam = new AprilTagWebcam();
        //sideCam = new AprilTagWebcam();

        hardwareInit(hardwareMap);
        telemetry.addLine("Hardware Initialised");
        turretCam.init(hardwareMap, telemetry,"Webcam 1");
        //sideCam.init(hardwareMap, telemetry, "Webcam 2");
        telemetry.addLine("Cameras Initialised");


        telemetry.addLine("Press to start...");
    }

    @Override
    public void loop(){

        if (gamepad1.dpadDownWasPressed()){active = !active;}//FIX

        if (active){

            turretCam.update();
            //sideCam.update();
            turretUpdate();
            //rotatedFromIdentity = getRotation();
            //telemetry.addData("Rotation", rotatedFromIdentity);

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

            //Intake
            boolean reverseIntake = gamepad1.a;
            if (reverseIntake) {
                intakeMotor.setPower(-(gamepad1.left_trigger));
            } else {
                intakeMotor.setPower(gamepad1.left_trigger);
            }
            // Flywheel
            if (gamepad1.right_trigger > 0.1) {
                flywheelMotorRight.setVelocity(1600);
                flywheelMotorLeft.setVelocity(1600)
            }

        }
        else{telemetry.addLine("Press Dpad-Down to activate");}
    }

    public void hardwareInit(HardwareMap hdwr){
        turret = hdwr.get(DcMotor.class, "servo1");//CHANGE TO dcMotor

        leftFrontMotor = hdwr.get(DcMotor.class, "frontLeft");
        leftBackMotor = hdwr.get(DcMotor.class, "backLeft");
        rightFrontMotor = hdwr.get(DcMotor.class, "frontRight");
        rightBackMotor = hdwr.get(DcMotor.class, "backRight");

        intakeMotor   = hdwr.get(DcMotor.class, "intakeMotor");
        flywheelMotor = hdwr.get(DcMotor.class, "flywheelMotor");

        flywheelMotorLeft = hdwr.get(DcMotor.class, "flywheelMotorLeft");
        flywheelMotorRight = hdwr.get(DcMotor.class, "flywheelMotorRight");

        flywheelMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//ADD
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        //imu = hardwareMap.get(IMU.class, "IMU");
        //IMU.Parameters  parameters;
        //parameters = new IMU.Parameters(
                //new RevHubOrientationOnRobot(
                        //RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        //RevHubOrientationOnRobot.UsbFacingDirection.FORWARD //Change for current robot
                //)
        //);
        //imu.resetYaw();

        //imu.initialize(parameters);

        //myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

    }

    public void turretUpdate(){
        AprilTagDetection goalTag = turretCam.getTagBySpecificId(goalTagID);

        if(goalTag != null){//UPDATE with deadzone
            Double degree = goalTag.ftcPose.bearing;
            if (degree > 3 || degree < -3){

            double power = -degree/20;
            turret.setPower(power);
            telemetry.addData("TurretRot", power);//SWITCH to encoders

            }
        }
        else{

            turret.setPower(0);
            telemetry.addData("TurretRot", 0);
        }
    }

    //public double getRotation(){
        //AprilTagDetection centerTag = sideCam.getTagBySpecificId(centerTagID);
        //AprilTagDetection goalTag = sideCam.getTagBySpecificId(goalTagID);
        //AprilTagDetection OppTag = sideCam.getTagBySpecificId(OppTagID);

        //if (centerTag == null && goalTag == null && OppTag == null){
            //double zRotationRate = myRobotAngularVelocity.zRotationRate;
            //return (rotatedFromIdentity + zRotationRate);
        //}

        //ArrayList<Double> angles = new ArrayList<Double>();

        //if(centerTag != null){
            //angles.add(centerTag.ftcPose.bearing - 90);
        //}


        //if(goalTag != null){
            //angles.add(goalTag.ftcPose.bearing + 45);
        //}


        //if(OppTag != null){
            //angles.add(OppTag.ftcPose.bearing - 45);
        //}

        //double totalAngle = 0;
        //for (int i = 0; i < angles.size(); i++) {
            //totalAngle += angles.get(i);
        //}

        //return (totalAngle/angles.size());
    //}
}
