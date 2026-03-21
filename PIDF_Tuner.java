package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PIDF Tuner")
public class TeleOp2026 extends OpMode{

    private DcMotorEx accelMotor;
    private double TargetVel, fastVel = 1600, slowVel = 900;
    private double p, f;
    private double[] step = {10, 1, 0.1, 0.001, 0.0001}
    private int stepIndex;

    @Override
    public void init(){
        
        motor = hardwareMap.get(DcMotorEx.class, "accel");
        accelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoeff = new PIDFCoefficients(p, 0, 0, f);
        accelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
    }

    @Override
    public void loop(){
       if (gamepad1.yWasPressed()){
            if (TargetVel == fastVel){
                TargetVel = slowVel;
            } else {TargetVel = fastVel;}
       }

       if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % step.length;
       }

       if (gamepad1.dpadLeftWasPressed()){
            f -= step[stepIndex];
       }

       if (gamepad1.dpadRightWasPressed()){
            f += step[stepIndex];
       }

       if (gamepad1.dpadUpWasPressed()){
            p += step[stepIndex];
       }

       if (gamepad1.dpadDownWasPressed()){
            p -= step[stepIndex];
       }

        PIDFCoefficients pidfCoeff = new PIDFCoefficients(p, 0, 0, f);
        accelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);

        motor.setVelocity(TargetVel);

        double curVel = motor.getVelocity();
        double error = TargetVel - curVel;

        telemetry.addData("Target Velocity", TargetVel);
        telemetry.addData("Current Velocity", curVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("---------------------------");
        telemetry.addData("P ", "%.4f", p);
        telemetry.addData("F", "%.4f", f);
        telemetry.addData("Step", "%.4f", step[stepIndex]);
    }
}
