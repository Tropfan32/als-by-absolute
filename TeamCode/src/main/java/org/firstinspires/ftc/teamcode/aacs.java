package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.adaptive.AdaptivePathPlanner;
import org.firstinspires.ftc.teamcode.adaptive.FoulPreventionBox;
import org.firstinspires.ftc.teamcode.adaptive.ImpactDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drivetrain curve check: Pedro {@link Follower} + Pinpoint follow one quadratic
 * Bézier, with AACS watching for collisions and injecting a recovery curve to
 * {@link #END}.
 *
 * <h2>Field setup</h2>
 * Place the robot at about {@code (24, 24)} inches, heading 0° (forward +X).
 * Needs clear space ~72 in forward and ~48 in left.
 *
 * <h2>Controls</h2>
 * PLAY starts the curve. Gamepad 1 <b>A</b> restarts it from the live pose.
 * Gamepad 1 <b>X</b> force-replans via AACS without a physical hit.
 */
@Autonomous(name = "Curve Drive Test", group = "Test")
public class aacs extends OpMode {

    private static final Pose START = new Pose(24, 24, 0);
    /**
     * Pulls the Bézier along +X before the left turn.
     */
    private static final Pose CONTROL = new Pose(96, 24, 0);
    private static final Pose END = new Pose(96, 72, Math.PI / 2.0);

    /**
     * Pedro reports accel in inches/s²; AACS ImpactDetector wants m/s².
     */
    private static final double INCHES_TO_METERS = 0.0254;

    private Follower follower;
    private Path curve;
    private boolean lastA;
    private boolean lastX;

    // ========== AACS (Autonomous Adaptive Control System) ==========
    private ImpactDetector impactDetector;
    private FoulPreventionBox geofence;
    private AdaptivePathPlanner planner;
    // ===============================================================

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);
        curve = buildCurve(START);

        // ----- AACS: build detector + geofence + planner -----
        impactDetector = new ImpactDetector();
        geofence = buildGeofence();
        planner = new AdaptivePathPlanner(follower, impactDetector);
        planner.setFoulPreventionBox(geofence);
        planner.setTargetPose(END);
        // ----- end AACS init -----

        telemetry.addLine("Curve Drive Test + AACS");
        telemetry.addLine("Hold still until PLAY. A = restart curve. X = AACS force replan.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Quadratic Bézier: (24,24) → (96,24) → (96,72)");
        telemetry.addData("start", poseString(START));
        telemetry.addData("end", poseString(END));
        telemetry.addData("AACS zones", geofence.getZoneCount());
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(curve, true);
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        if (a && !lastA) {
            Pose now = follower.getPose();
            curve = buildCurve(now);
            follower.followPath(curve, true);
        }
        lastA = a;

        boolean x = gamepad1.x;
        if (x && !lastX) {
            // AACS: inject recovery Bézier to END without waiting for a hit
            planner.forceReplan();
        }
        lastX = x;

        // ----- AACS loop contract (do not reorder) -----
        Vector accelInches = follower.getAcceleration();
        double ax = accelInches.getXComponent() * INCHES_TO_METERS;
        double ay = accelInches.getYComponent() * INCHES_TO_METERS;
        impactDetector.update(ax, ay); // AACS: latch collision from Pinpoint accel
        planner.update();              // AACS: on hit, inject recovery Bézier to END
        follower.update();             // Pedro: execute current path
        // ----- end AACS loop -----

        Pose pose = follower.getPose();
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("pose", poseString(pose));
        telemetry.addData("heading deg", Math.toDegrees(pose.getHeading()));
        telemetry.addLine("--- AACS ---");
        telemetry.addData("impact", impactDetector.isImpactDetected());
        telemetry.addData("a m/s^2 |A| g", String.format("%.2f",
                impactDetector.getLastHorizontalAccelG()));
        telemetry.addData("AACS replans", planner.getReplanCount());
        telemetry.addData("AACS cooldown", planner.isCoolingDown());
        telemetry.addLine("A = restart curve  X = AACS force replan");
        telemetry.update();
    }

    /**
     * Quadratic Bézier from {@code from} through {@link #CONTROL} to {@link #END}.
     * Tangent heading keeps the chassis aligned with the curve.
     */
    private static Path buildCurve(Pose from) {
        Path path = new Path(new BezierCurve(from, CONTROL, END));
        path.setTangentHeadingInterpolation();
        return path;
    }

    /**
     * AACS: field walls + sample illegal boxes so recovery curves stay legal.
     */
    private static FoulPreventionBox buildGeofence() {
        FoulPreventionBox box = new FoulPreventionBox();
        box.setFieldBounds(0.0, 0.0, FoulPreventionBox.PEDRO_FIELD_INCHES,
                FoulPreventionBox.PEDRO_FIELD_INCHES);
        box.setClearanceInches(6.0);
        box.addAabb("opponent_wing", 0.0, 0.0, 24.0, 48.0);
        box.addAabb("submersible", 54.0, 54.0, 90.0, 90.0);
        return box;
    }

    private static String poseString(Pose pose) {
        if (pose == null) {
            return "null";
        }
        return String.format("(%.1f, %.1f, %.0f°)",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
