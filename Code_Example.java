//This is code i got gemini to do just for the overall look and layout of Pedro Pathing code
//U can still look into better examples though but i just have it on my CodeHS files



@Autonomous(name = "Pedro_Auto")
public class PedroAuto extends OpMode {

    // 1. VARIABLES
    private Follower follower;
    private DcMotorEx flywheel;
    private PathChain scorePath; // A linked sequence of paths
    
    // 2. POSES (Coordinates)
    // Remember: Pedro uses a 0-144 inch grid. (0,0) is a corner, not the center.
    private final Pose startPose = new Pose(8, 60, Math.toRadians(0));
    private final Pose targetPose = new Pose(30, 60, Math.toRadians(0));

    

    @Override
    public void init() {
        // 3. HARDWARE & FOLLOWER SETUP
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // 4. BUILD THE PATHS (You copy-paste this from the Visualizer!)
        scorePath = follower.pathBuilder()
            .addPath(new BezierLine(new Point(startPose), new Point(targetPose)))
            .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
            .build();
    }

    @Override
    public void start() {
        // This runs EXACTLY once when you hit the Play triangle
        // We tell the follower to start driving immediately
        follower.followPath(scorePath);
    }

    @Override
    public void loop() {
        // 5. THE HEARTBEAT
        // This MUST be in your loop. It updates the motors and reads the Pinpoint.
        follower.update();

        // 6. THE FLYWHEEL LOGIC (State Machine)
        // Since we are in a loop, we check the robot's X coordinate.
        // If the robot has driven past X=15, spin the flywheel!
        if (follower.getPose().getX() > 15) {
            flywheel.setPower(1.0);
        } else {
            flywheel.setPower(0.0);
        }

        // Telemetry to watch on the driver station
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
}
