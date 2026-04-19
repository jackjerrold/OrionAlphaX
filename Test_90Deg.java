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

@Autonomous(name = "90Deg_Test", group = "Autonomous")
@Configurable // Panels
public class Test_90Deg extends OpMode { // FIX 1: Renamed class to not start with a number

  private TelemetryManager panelsTelemetry;
  public Follower follower;
  private int pathState = 0; // Start at state 0
  private Paths paths;

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    follower = Constants.createFollower(hardwareMap);
    
    // FIX 2: Updated Starting Pose to match the very first point of your BezierLine (121, 123)
    // and the starting heading of 37 degrees.
    follower.setStartingPose(new Pose(121.000, 123.000, Math.toRadians(37)));

    paths = new Paths(follower);

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  // FIX 3: Added the start() method. This runs exactly ONCE when you hit the "Play" button.
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

  // FIX 4: Implemented the state machine
  public int autonomousPathUpdate() {
    switch (pathState) {
        case 1:
            // We are in State 1. The robot is currently following the MainChain.
            // We check if it is done. If it isn't busy anymore, move to State 2.
            if (!follower.isBusy()) {
                pathState = 2; 
            }
            break;
            
        case 2:
            // The path is complete! 
            // In the future, you could trigger a servo, start an intake, 
            // or tell it to follow another path here, and then set pathState = 3.
            break;
    }
    return pathState;
  }
}
