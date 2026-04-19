package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Test_90Deg", group = "Autonomous")
@Configurable // Panels
public class Test_90Deg extends OpMode {

  private TelemetryManager panelsTelemetry;
  public Follower follower;
  private int pathState = 0; // Start at state 0
  private Paths paths;
  private com.qualcomm.robotcore.util.ElapsedTime stateTimer = new com.qualcomm.robotcore.util.ElapsedTime();

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    follower = Constants.createFollower(hardwareMap);

    
    follower.setStartingPose(new Pose(121.000, 123.000, Math.toRadians(37)));

    paths = new Paths(follower);

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }


  @Override
  public void start() {
    follower.followPath(paths.MainChain); // Tell the robot to start driving!
    pathState = 1; // Move the state machine to state 1 (Following path)
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing motor powers and location
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading())); // Converted to degrees for easier reading
    panelsTelemetry.update(telemetry);
  }

  public static class Paths {
    public PathChain MainChain;

    public Paths(Follower follower) {
      MainChain = follower.pathBuilder()
          .addPath(
            new BezierLine(
              new Pose(121.000, 123.000),
              new Pose(85.000, 85.000)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(-90))
          .addPath(
            new BezierLine(
              new Pose(85.000, 85.000),
              new Pose(85.000, 15.000)
            )
          )
          .setConstantHeadingInterpolation(Math.toRadians(-90))
          .build();
    }
  }


  public int autonomousPathUpdate() {
      switch (pathState) {
          case 1:
              // Check if the robot has reached the end of 'MainChain'
              if (!follower.isBusy()) {
                  stateTimer.reset(); // Start the clock the moment the path ends
                  pathState = 2;
                  telemetry.addLine(!follower.isBusy());
              }
              break;
            
          case 2:
              telemetry.addLine(stateTimer.seconds());
              if (stateTimer.seconds() >= 1.0) {
                  pathState = 3; 
              }
              break;

          case 3:
              telemetry.addLine("Movement Finished");
              break;
      }
      return pathState;
  }
}
