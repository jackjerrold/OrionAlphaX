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
public class Test_90Deg extends OpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(121, 123, Math.toRadians(90)));

    paths = new Paths(follower); // Build paths

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
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
    // Add your state machine Here
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return 0;
  }
}
