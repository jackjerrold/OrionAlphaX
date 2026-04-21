package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

// ============================================================
//  DECODE 2025/26 — Full Autonomous OpMode
//  Includes: AprilTag localisation, wheel odometry, IMU fusion,
//            A* grid pathfinding, PID mecanum motion control,
//            DECODE state machine (Leave → Collect → Score → Park)
//
//  BEFORE YOUR FIRST RUN — search "TUNE ME" for all values
//  you must measure/adjust on your actual robot.
// ============================================================
@Autonomous(name = "DECODE Auto", group = "EmergencyAuto")
public class DecodeAuto extends LinearOpMode {

    // --------------------------------------------------------
    // ALLIANCE SELECTION — set before match via gamepad on init
    // --------------------------------------------------------
    private boolean isRedAlliance = true; // changed by gamepad during init()

    // --------------------------------------------------------
    // HARDWARE — TUNE ME: change names to match your config
    // --------------------------------------------------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private IMU imu;

    // --------------------------------------------------------
    // VISION
    // --------------------------------------------------------
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // AprilTag world positions on DECODE field (inches from field centre)
    // TUNE ME: measure your actual tag placements on the field
    // Tag 0 = near starting wall, Tag 1 = near goal, Tag 2 = near base/park
    // Format: { worldX, worldY } in inches, Red-alliance frame
    // (For Blue alliance these are mirrored automatically below)
    private static final double[][] TAG_WORLD_POS_RED = {
        {  60.0,  -36.0 },  // Tag 0 — near back wall
        {  20.0,  -48.0 },  // Tag 1 — near goal/shooting spot
        {  60.0,    0.0 }   // Tag 2 — near base/park zone
    };

    // --------------------------------------------------------
    // FIELD GRID
    // FTC field = 144" x 144", origin at centre.
    // Grid cell size = 6 inches → 24x24 grid.
    // Grid (0,0) = field corner at (-72, -72) in world coords.
    // --------------------------------------------------------
    private static final double CELL_IN  = 6.0;   // inches per cell
    private static final int    GRID_N   = 24;     // cells per side

    private final boolean[][] blocked = new boolean[GRID_N][GRID_N];

    // --------------------------------------------------------
    // POSE  (inches, degrees — world frame, centre origin)
    // --------------------------------------------------------
    private double poseX   = 0;
    private double poseY   = 0;
    private double poseHdg = 0; // degrees, 0 = toward audience wall (+X)

    // --------------------------------------------------------
    // ODOMETRY
    // TUNE ME: measure your wheel circumference and encoder PPR
    // --------------------------------------------------------
    private static final double WHEEL_DIAM_IN   = 3.78;   // 96mm mecanum wheel
    private static final double ENCODER_PPR      = 537.7;  // GoBilda 312 RPM motor
    private static final double INCHES_PER_TICK  = (Math.PI * WHEEL_DIAM_IN) / ENCODER_PPR;
    private static final double TRACK_WIDTH_IN   = 13.5;  // TUNE ME: left-to-right wheel distance

    private int prevLF, prevRF, prevLB, prevRB;

    // --------------------------------------------------------
    // MOTION PID
    // TUNE ME: start with small Kp, increase until robot reaches
    // targets smoothly without overshooting
    // --------------------------------------------------------
    private static final double DRIVE_KP        = 0.035;  // power per inch of error
    private static final double TURN_KP         = 0.012;  // power per degree of error
    private static final double MAX_DRIVE_POWER = 0.6;
    private static final double MAX_TURN_POWER  = 0.4;
    private static final double ARRIVE_THRESH   = 3.0;    // inches — "close enough"
    private static final double ANGLE_THRESH    = 3.0;    // degrees — "aligned enough"

    // --------------------------------------------------------
    // DECODE FIELD TARGETS  (Red alliance — Blue mirrors below)
    // TUNE ME: verify against your actual field setup
    // --------------------------------------------------------
    // Starting position (robot centre, placed against back wall)
    private static final double START_X_RED = -60.0, START_Y_RED = -36.0;

    // Launch Line cross point (just needs to be past the line)
    private static final double LAUNCH_X_RED = -36.0, LAUNCH_Y_RED = -36.0;

    // Spike mark centres (near, mid, far)
    private static final double[][] SPIKE_RED = {
        { 24.0, -48.0 },  // near spike (audience side)
        { 24.0, -24.0 },  // mid spike
        { 24.0,   0.0 }   // far spike (goal side)
    };

    // Shooting position in front of goal
    private static final double SHOOT_X_RED = 0.0, SHOOT_Y_RED = -48.0;

    // Base/park zone
    private static final double PARK_X_RED = -48.0, PARK_Y_RED = -12.0;

    // --------------------------------------------------------
    // STATE MACHINE
    // --------------------------------------------------------
    private enum AutoState {
        INIT_POSE,
        CROSS_LAUNCH_LINE,
        SEEK_APRILTAG_FIX,
        NAVIGATE_TO_SPIKE,
        COLLECT_ARTIFACTS,    // STUB — add your intake code here
        NAVIGATE_TO_SHOOT,
        SHOOT,                // STUB — add your shooter code here
        NAVIGATE_TO_PARK,
        PARK,
        DONE
    }
    private AutoState state = AutoState.INIT_POSE;

    // Current path being followed
    private List<int[]> currentPath = new ArrayList<>();
    private int waypointIdx = 0;

    // Which spike we're heading to (0 = near, 1 = mid, 2 = far)
    private int spikeTarget = 0;

    // Track how many shoot attempts done
    private int shootCount = 0;

    private final ElapsedTime stateTimer = new ElapsedTime();

    // ============================================================
    //  MAIN
    // ============================================================
    @Override
    public void runOpMode() {

        // --- Hardware init ---
        initHardware();
        initVision();
        initGrid();

        // --- Alliance selection during init phase ---
        telemetry.addLine("Press X = RED alliance,  B = BLUE alliance");
        telemetry.update();
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) { isRedAlliance = true; }
            if (gamepad1.b) { isRedAlliance = false; }
            telemetry.addLine("Alliance: " + (isRedAlliance ? "RED" : "BLUE"));
            telemetry.addLine("Press PLAY to start");
            telemetry.update();
            sleep(50);
        }

        waitForStart();
        if (isStopRequested()) return;

        // Set starting pose based on alliance
        if (isRedAlliance) {
            poseX = START_X_RED;
            poseY = START_Y_RED;
            poseHdg = 90.0; // facing toward centre field
        } else {
            poseX = -START_X_RED;  // mirror X for blue
            poseY = -START_Y_RED;
            poseHdg = -90.0;
        }

        saveEncoders();
        stateTimer.reset();

        // ---- Main loop ----
        while (opModeIsActive()) {
            updateOdometry();
            tryAprilTagFix();
            runStateMachine();
            sendTelemetry();
        }

        stopDrive();
        visionPortal.close();
    }

    // ============================================================
    //  STATE MACHINE
    // ============================================================
    private void runStateMachine() {
        switch (state) {

            // ── Set pose already done above, skip straight through ──
            case INIT_POSE:
                transition(AutoState.CROSS_LAUNCH_LINE);
                break;

            // ── Drive past the launch line for the 3-pt bonus ──
            case CROSS_LAUNCH_LINE: {
                double tx = isRedAlliance ? LAUNCH_X_RED : -LAUNCH_X_RED;
                double ty = isRedAlliance ? LAUNCH_Y_RED : -LAUNCH_Y_RED;
                if (driveToWorld(tx, ty)) {
                    transition(AutoState.SEEK_APRILTAG_FIX);
                }
                break;
            }

            // ── Pause briefly to let camera lock onto a tag ──
            case SEEK_APRILTAG_FIX:
                stopDrive();
                tryAprilTagFix();
                if (stateTimer.seconds() > 0.8) {
                    transition(AutoState.NAVIGATE_TO_SPIKE);
                }
                break;

            // ── Navigate to the nearest spike mark ──
            case NAVIGATE_TO_SPIKE: {
                double[] spike = getSpikeTarget(spikeTarget);
                if (driveToWorld(spike[0], spike[1])) {
                    transition(AutoState.COLLECT_ARTIFACTS);
                }
                break;
            }

            // ── STUB: run your intake mechanism here ──
            // Replace the timed wait with your actual intake logic.
            case COLLECT_ARTIFACTS:
                stopDrive();
                // TODO: startIntake();
                if (stateTimer.seconds() > 1.5) {
                    // TODO: stopIntake();
                    transition(AutoState.NAVIGATE_TO_SHOOT);
                }
                break;

            // ── Drive to shooting position ──
            case NAVIGATE_TO_SHOOT: {
                double tx = isRedAlliance ? SHOOT_X_RED : -SHOOT_X_RED;
                double ty = isRedAlliance ? SHOOT_Y_RED : -SHOOT_Y_RED;
                if (driveToWorld(tx, ty)) {
                    // Align heading toward goal before shooting
                    double targetHdg = isRedAlliance ? 0.0 : 180.0;
                    if (turnToHeading(targetHdg)) {
                        transition(AutoState.SHOOT);
                    }
                }
                break;
            }

            // ── STUB: fire your shooter here ──
            // Replace timed wait with actual shooter control.
            case SHOOT:
                stopDrive();
                // TODO: runShooter();
                if (stateTimer.seconds() > 2.0) {
                    // TODO: stopShooter();
                    shootCount++;
                    // If time remains and we haven't done all spikes, do another
                    if (shootCount < 2 && spikeTarget < 2) {
                        spikeTarget++;
                        transition(AutoState.NAVIGATE_TO_SPIKE);
                    } else {
                        transition(AutoState.NAVIGATE_TO_PARK);
                    }
                }
                break;

            // ── Drive to base/park zone ──
            case NAVIGATE_TO_PARK: {
                double tx = isRedAlliance ? PARK_X_RED : -PARK_X_RED;
                double ty = isRedAlliance ? PARK_Y_RED : -PARK_Y_RED;
                if (driveToWorld(tx, ty)) {
                    transition(AutoState.PARK);
                }
                break;
            }

            case PARK:
                stopDrive();
                transition(AutoState.DONE);
                break;

            case DONE:
                stopDrive();
                break;
        }
    }

    private void transition(AutoState next) {
        state = next;
        currentPath.clear();
        waypointIdx = 0;
        stateTimer.reset();
    }

    // ============================================================
    //  MOTION — drive to a world coordinate via A* grid path
    // ============================================================

    /**
     * Call repeatedly each loop. Returns true when arrived.
     */
    private boolean driveToWorld(double targetX, double targetY) {
        // Re-plan if path is empty or first call for this target
        if (currentPath.isEmpty()) {
            int sr = worldToGrid(poseY), sc = worldToGrid(poseX);
            int gr = worldToGrid(targetY), gc = worldToGrid(targetX);
            currentPath = astar(sr, sc, gr, gc);
            if (currentPath == null || currentPath.isEmpty()) {
                // No path found — drive direct
                currentPath = new ArrayList<>();
                currentPath.add(new int[]{gr, gc});
            }
            waypointIdx = 0;
        }

        // Advance through waypoints
        while (waypointIdx < currentPath.size()) {
            int[] wp = currentPath.get(waypointIdx);
            double wpX = gridToWorld(wp[1]);
            double wpY = gridToWorld(wp[0]);
            double dist = Math.hypot(wpX - poseX, wpY - poseY);
            if (dist < ARRIVE_THRESH) {
                waypointIdx++;
            } else {
                applyMecanumDrive(wpX, wpY);
                return false;
            }
        }

        // All waypoints done — final approach to exact target
        double dist = Math.hypot(targetX - poseX, targetY - poseY);
        if (dist < ARRIVE_THRESH) {
            stopDrive();
            return true;
        }
        applyMecanumDrive(targetX, targetY);
        return false;
    }

    /**
     * PID mecanum drive toward a world point.
     * Decomposes error into robot-local forward/strafe, adds heading correction.
     */
    private void applyMecanumDrive(double targetX, double targetY) {
        double errX  = targetX - poseX;
        double errY  = targetY - poseY;
        double hdgRad = Math.toRadians(poseHdg);

        // Rotate world error into robot frame
        double fwd    =  errX * Math.cos(hdgRad) + errY * Math.sin(hdgRad);
        double strafe = -errX * Math.sin(hdgRad) + errY * Math.cos(hdgRad);

        // Heading — keep robot pointing at target while driving
        double desiredHdg  = Math.toDegrees(Math.atan2(errY, errX));
        double hdgErr       = normalizeAngle(desiredHdg - poseHdg);

        double drivePow = clip(DRIVE_KP * Math.hypot(fwd, strafe), MAX_DRIVE_POWER);
        double fwdPow   = drivePow * (fwd   / Math.hypot(fwd, strafe + 1e-9));
        double strafePow= drivePow * (strafe/ Math.hypot(fwd + 1e-9, strafe));
        double turnPow  = clip(TURN_KP * hdgErr, MAX_TURN_POWER);

        setMecanum(fwdPow, strafePow, turnPow);
    }

    /**
     * Rotate in place to a heading. Returns true when aligned.
     */
    private boolean turnToHeading(double targetHdg) {
        double err = normalizeAngle(targetHdg - poseHdg);
        if (Math.abs(err) < ANGLE_THRESH) {
            stopDrive();
            return true;
        }
        double p = clip(TURN_KP * err, MAX_TURN_POWER);
        setMecanum(0, 0, p);
        return false;
    }

    private void setMecanum(double fwd, double strafe, double turn) {
        double lf = fwd + strafe + turn;
        double rf = fwd - strafe - turn;
        double lb = fwd - strafe + turn;
        double rb = fwd + strafe - turn;
        double max = Math.max(1.0, Math.max(Math.abs(lf),
                     Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }

    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // ============================================================
    //  ODOMETRY — wheel encoder dead reckoning
    // ============================================================
    private void updateOdometry() {
        int lf = leftFront.getCurrentPosition();
        int rf = rightFront.getCurrentPosition();
        int lb = leftBack.getCurrentPosition();
        int rb = rightBack.getCurrentPosition();

        double dLF = (lf - prevLF) * INCHES_PER_TICK;
        double dRF = (rf - prevRF) * INCHES_PER_TICK;
        double dLB = (lb - prevLB) * INCHES_PER_TICK;
        double dRB = (rb - prevRB) * INCHES_PER_TICK;

        prevLF = lf; prevRF = rf; prevLB = lb; prevRB = rb;

        // Use IMU as primary heading source (more reliable than encoder-derived)
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        poseHdg = angles.getYaw(AngleUnit.DEGREES);

        // Forward displacement (average of all four wheels)
        double fwd = (dLF + dRF + dLB + dRB) / 4.0;

        // Strafe displacement (mecanum kinematics)
        double strafe = (-dLF + dRF + dLB - dRB) / 4.0;

        // Integrate in world frame
        double hdgRad = Math.toRadians(poseHdg);
        poseX += fwd * Math.cos(hdgRad) - strafe * Math.sin(hdgRad);
        poseY += fwd * Math.sin(hdgRad) + strafe * Math.cos(hdgRad);
    }

    private void saveEncoders() {
        prevLF = leftFront.getCurrentPosition();
        prevRF = rightFront.getCurrentPosition();
        prevLB = leftBack.getCurrentPosition();
        prevRB = rightBack.getCurrentPosition();
    }

    // ============================================================
    //  APRILTAG LOCALISATION
    //  ftcPose gives camera-relative position of the tag.
    //  We invert to get robot position in world frame.
    // ============================================================
    private void tryAprilTagFix() {
        if (aprilTagProcessor == null) return;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection det : detections) {
            if (det.metadata == null) continue;
            int id = det.id;
            if (id < 0 || id > 2) continue;

            double[] tagWorld = getTagWorldPos(id);

            // ftcPose: x = right, y = forward, z = up from camera, in CM → convert to inches
            double camFwdIn  = det.ftcPose.y / 2.54;  // forward from camera
            double camRightIn= det.ftcPose.x / 2.54;  // right from camera
            double tagYawDeg = det.ftcPose.yaw;         // tag yaw seen from camera

            // Robot heading from this detection
            // When camera faces the tag straight-on, yaw=0 and robot heading = tag's known facing
            // TUNE ME: tagFacingDeg = the direction the tag faces on the field (degrees)
            double tagFacingDeg = isRedAlliance ? 180.0 : 0.0; // tags face toward centre
            double robotHdg = normalizeAngle(tagFacingDeg + tagYawDeg);

            // Camera offset from robot centre
            // TUNE ME: how far forward the camera is from robot centre (inches)
            double CAM_FWD_OFFSET = 6.0;
            double hdgRad = Math.toRadians(robotHdg);

            // Robot position = tag world pos minus vector from robot to tag (in world frame)
            double robotX = tagWorld[0]
                    - camFwdIn  * Math.cos(hdgRad)
                    + camRightIn * Math.sin(hdgRad)
                    - CAM_FWD_OFFSET * Math.cos(hdgRad);
            double robotY = tagWorld[1]
                    - camFwdIn  * Math.sin(hdgRad)
                    - camRightIn * Math.cos(hdgRad)
                    - CAM_FWD_OFFSET * Math.sin(hdgRad);

            // Only accept fix if it's plausible (within 18" of current dead-reckoned pos)
            double drift = Math.hypot(robotX - poseX, robotY - poseY);
            if (drift < 18.0) {
                poseX   = robotX;
                poseY   = robotY;
                poseHdg = robotHdg;
                saveEncoders(); // reset accumulator from new known pose
            }
        }
    }

    private double[] getTagWorldPos(int id) {
        if (isRedAlliance) {
            return TAG_WORLD_POS_RED[id];
        } else {
            // Mirror X axis for blue alliance
            return new double[]{ -TAG_WORLD_POS_RED[id][0], -TAG_WORLD_POS_RED[id][1] };
        }
    }

    // ============================================================
    //  A* PATHFINDER
    // ============================================================
    private static class Node {
        int row, col;
        double g, f;
        Node parent;
        Node(int r, int c, double g, double f, Node p) {
            row=r; col=c; this.g=g; this.f=f; parent=p;
        }
    }

    private List<int[]> astar(int sr, int sc, int gr, int gc) {
        if (!inBounds(sr,sc) || !inBounds(gr,gc)) return new ArrayList<>();
        double[][] gScore = new double[GRID_N][GRID_N];
        for (double[] row : gScore) java.util.Arrays.fill(row, Double.MAX_VALUE);
        gScore[sr][sc] = 0;

        List<Node> open = new ArrayList<>();
        open.add(new Node(sr, sc, 0, heuristic(sr,sc,gr,gc), null));

        boolean[][] closed = new boolean[GRID_N][GRID_N];

        int[][] dirs = {{0,1},{0,-1},{1,0},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}};

        while (!open.isEmpty()) {
            // Pop lowest f
            Node cur = open.get(0);
            for (Node n : open) if (n.f < cur.f) cur = n;
            open.remove(cur);

            if (cur.row == gr && cur.col == gc) return reconstructPath(cur);
            closed[cur.row][cur.col] = true;

            for (int[] d : dirs) {
                int nr = cur.row + d[0], nc = cur.col + d[1];
                if (!inBounds(nr,nc) || blocked[nr][nc] || closed[nr][nc]) continue;
                double cost = (Math.abs(d[0])+Math.abs(d[1]) == 2) ? 1.414 : 1.0;
                double ng = cur.g + cost;
                if (ng < gScore[nr][nc]) {
                    gScore[nr][nc] = ng;
                    double f = ng + heuristic(nr,nc,gr,gc);
                    open.add(new Node(nr, nc, ng, f, cur));
                }
            }
        }
        return new ArrayList<>(); // no path
    }

    private List<int[]> reconstructPath(Node node) {
        List<int[]> path = new ArrayList<>();
        while (node != null) { path.add(new int[]{node.row, node.col}); node = node.parent; }
        Collections.reverse(path);
        return smoothPath(path);
    }

    /** Remove collinear intermediate waypoints to reduce jitter */
    private List<int[]> smoothPath(List<int[]> raw) {
        if (raw.size() <= 2) return raw;
        List<int[]> smooth = new ArrayList<>();
        smooth.add(raw.get(0));
        for (int i = 1; i < raw.size()-1; i++) {
            int[] prev = raw.get(i-1), cur = raw.get(i), next = raw.get(i+1);
            boolean sameDir = (cur[0]-prev[0]) == (next[0]-cur[0])
                           && (cur[1]-prev[1]) == (next[1]-cur[1]);
            if (!sameDir) smooth.add(cur);
        }
        smooth.add(raw.get(raw.size()-1));
        return smooth;
    }

    private double heuristic(int r1, int c1, int r2, int c2) {
        return Math.sqrt((r2-r1)*(r2-r1) + (c2-c1)*(c2-c1));
    }

    // ============================================================
    //  GRID UTILITIES
    // ============================================================

    /** World inches (centre origin) → grid index */
    private int worldToGrid(double worldInches) {
        return (int) Math.max(0, Math.min(GRID_N-1,
                Math.floor((worldInches + 72.0) / CELL_IN)));
    }

    /** Grid index → world inches (cell centre) */
    private double gridToWorld(int idx) {
        return idx * CELL_IN + CELL_IN / 2.0 - 72.0;
    }

    private boolean inBounds(int r, int c) {
        return r >= 0 && r < GRID_N && c >= 0 && c < GRID_N;
    }

    private void markBlocked(int r, int c) {
        if (inBounds(r, c)) blocked[r][c] = true;
    }

    /** Pre-mark known DECODE field obstacles */
    private void initGrid() {
        // Always block perimeter cells
        for (int i = 0; i < GRID_N; i++) {
            blocked[0][i] = blocked[GRID_N-1][i] = true;
            blocked[i][0] = blocked[i][GRID_N-1] = true;
        }

        // DECODE: central goal structure (~3 tiles wide, centred at field centre)
        // TUNE ME: verify against your field layout
        // Field centre = grid cell (12, 12). Goal footprint approx rows 10-14, cols 10-14
        for (int r = 10; r <= 14; r++)
            for (int c = 10; c <= 14; c++)
                markBlocked(r, c);
    }

    // ============================================================
    //  SPIKE TARGET HELPER
    // ============================================================
    private double[] getSpikeTarget(int idx) {
        double[] red = SPIKE_RED[Math.min(idx, SPIKE_RED.length-1)];
        if (isRedAlliance) return red;
        return new double[]{ -red[0], -red[1] };
    }

    // ============================================================
    //  HARDWARE INIT
    // ============================================================
    private void initHardware() {
        // TUNE ME: rename these to match your hardware config names
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // TUNE ME: flip direction for motors that spin backward
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU — TUNE ME: set your Control Hub orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(imuParams);
        imu.resetYaw();
    }

    // ============================================================
    //  VISION INIT
    // ============================================================
    private void initVision() {
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
            .addTag(0, "Tag 0", 0.0508, DistanceUnit.METER)
            .addTag(1, "Tag 1", 0.0508, DistanceUnit.METER)
            .addTag(2, "Tag 2", 0.0508, DistanceUnit.METER)
            .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(tagLibrary)
            // TUNE ME: use your camera's actual calibration values
            .setLensIntrinsics(578, 578, 320, 240)
            .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
            .build();

        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
            .setCameraResolution(new Size(640, 480))
            .addProcessor(aprilTagProcessor)
            .build();
    }

    // ============================================================
    //  TELEMETRY
    // ============================================================
    private void sendTelemetry() {
        telemetry.addLine("=== DECODE AUTO ===");
        telemetry.addData("Alliance",  isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("State",     state);
        telemetry.addData("Pose",      "X:%.1f Y:%.1f Hdg:%.1f", poseX, poseY, poseHdg);
        telemetry.addData("Grid cell", "(%d, %d)",
                worldToGrid(poseX), worldToGrid(poseY));
        telemetry.addData("Waypoint",  "%d / %d", waypointIdx, currentPath.size());
        telemetry.addData("Spike #",   spikeTarget);
        telemetry.addData("Tags seen", aprilTagProcessor.getDetections().size());
        telemetry.update();
    }

    // ============================================================
    //  MATH HELPERS
    // ============================================================
    private double normalizeAngle(double deg) {
        while (deg >  180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private double clip(double val, double max) {
        return Math.max(-max, Math.min(max, val));
    }
}
