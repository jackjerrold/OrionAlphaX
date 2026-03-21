package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@Autonomous
public class CameraTest extends OpMode{

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init(){
        aprilTagWebcam.init(hardwareMap, telemetry,"webcam 1");
    }

    @Override
    public void loop(){
        aprilTagWebcam.update();
        AprilTagDetection id01 = aprilTagWebcam.getTagBySpecificId(1);
        aprilTagWebcam.displayDetectionTelemetry(id01);
    }
}