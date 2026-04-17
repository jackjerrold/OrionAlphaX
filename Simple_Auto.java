/*
This code is basic and for having options and atleast something for auto (90% will work)

This code is for just stepping back, shooting, and strafe off the line 
This is for next to the goal start

All the functions and stuff before [while opmodeactive()] is my old code so it worked before.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Simple Auto")
public class Simple_Auto extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx flywheelMotor;


    // Flywheel PIDF for velocity control
    private final double flywheelTargetRPM = 6000;
    private final double flywheelP = 0.1;
    private final double flywheelF = 12.0 / flywheelTargetRPM;
    
    static final double TICKS_PER_REV = 28;
    static final double WHEEL_DIAMETER = 104; //Millimeters
    static final double GEAR_RATIO = 20;
    
    static final double TICKS_PER_REV_FLYWHEEL = 28;
    static final double GEAR_RATIO_FLYWHEEL = 0.926;

    public void setFlywheelRPM(double rpm) {
    double ticksPerSecond = (rpm*GEAR_RATIO_FLYWHEEL / 60.0) * TICKS_PER_REV_FLYWHEEL;
    flywheelMotor.setVelocity(ticksPerSecond);
}

    public void encoderDrive(double speed, double distanceMM) {

        int ticks = (int) (
                (distanceMM / (Math.PI * WHEEL_DIAMETER))
                * TICKS_PER_REV
                * GEAR_RATIO
        );

        int frontLeftTarget = frontLeft.getCurrentPosition() + ticks;
        int frontRightTarget = frontRight.getCurrentPosition() + ticks;
        int backLeftTarget = backLeft.getCurrentPosition() + ticks;
        int backRightTarget = backRight.getCurrentPosition() + ticks;

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&
            (frontLeft.isBusy() || frontRight.isBusy() ||
            backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    public void strafe(double speed, double distanceMM) {

    int ticks = (int) ((distanceMM / (Math.PI * WHEEL_DIAMETER)) * TICKS_PER_REV * GEAR_RATIO);

    frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
    frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
    backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
    backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    frontLeft.setPower(Math.abs(speed));
    frontRight.setPower(Math.abs(speed));
    backLeft.setPower(Math.abs(speed));
    backRight.setPower(Math.abs(speed));


    while (opModeIsActive() &&
        (frontLeft.isBusy() || frontRight.isBusy() ||
        backLeft.isBusy() || backRight.isBusy())) {

        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.update();
    }

    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);

    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    public void rotate(double speed, double angleDegrees) {

    double robotWidth = 435;
    double wheelDistance = (robotWidth * Math.PI) * (angleDegrees / 360.0);

    int ticks = (int)((wheelDistance / (Math.PI * WHEEL_DIAMETER)) * TICKS_PER_REV * GEAR_RATIO);

    frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
    frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
    backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
    backRight.setTargetPosition(backRight.getCurrentPosition() - ticks);

    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    frontLeft.setPower(Math.abs(speed));
    frontRight.setPower(Math.abs(speed));
    backLeft.setPower(Math.abs(speed));
    backRight.setPower(Math.abs(speed));

    while (opModeIsActive() &&
        (frontLeft.isBusy() || frontRight.isBusy() ||
        backLeft.isBusy() || backRight.isBusy())) {

        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.update();
    }

    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);

    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    @Override
    public void runOpMode() throws InterruptedException {

    
        frontLeft  = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft   = hardwareMap.get(DcMotorEx.class, "lb");
        backRight  = hardwareMap.get(DcMotorEx.class, "rb");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelP, 0, 0, flywheelF); 
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        
        double driveSpeed = 0.6; 
        double slowSpeed = 0.3;

        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        if(opModeIsActive()) {
          telemetry.addLine("Rotating 360 at 0.6 speed");
          telemetry.update();
          rotate(driveSpeed, 360);

          int tims = 5;
          while (tims > 0) {
            telemetry.addLine("start in time:" + tims);
            telemetry.update();
            sleep(1000);
            tims = tims - 1;
          }

          
          encoderDrive(-driveSpeed, 500);
            
          setFlywheelRPM(4000);
          intakeMotor.setPower(1);
            
          sleep(5000);
          
          setFlywheelRPM(0);
          intakeMotor.setPower(0);
          
          strafe(driveSpeed, 500);
        }
    }
}
