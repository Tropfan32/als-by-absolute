package org.firstinspires.ftc.teamcode.adaptive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Driver-station test for the full adaptive stack: Pinpoint → {@link ImpactDetector}
 * → {@link AdaptivePathPlanner} → Pedro {@link Follower}, with
 * {@link FoulPreventionBox} geofencing on recovery Béziers.
 *
 * <h2>How to run</h2>
 * <ol>
 *   <li>Configure the goBILDA Pinpoint I2C device as {@code pinpoint}. Pod offsets
 *       are {@code (-120 mm, -150 mm)} in {@link Constants#localizerConstants}.</li>
 *   <li>Place the robot on the starting pose (default Pedro inches
 *       {@code (36, 24, 0°)}). Keep it still through {@code init} so Pinpoint’s
 *       IMU can settle.</li>
 *   <li>Select <b>Adaptive Pipeline Test</b> (TeleOp group {@code Adaptive}).</li>
 *   <li>Press PLAY, then use gamepad 1:
 *       <ul>
 *         <li><b>A</b> — set recovery goal to Target Alpha and follow a straight
 *             path there.</li>
 *         <li><b>B</b> — set recovery goal to Target Beta (does not start a new
 *             path by itself; the next impact / force-replan uses Beta).</li>
 *         <li><b>X</b> — {@link AdaptivePathPlanner#forceReplan()} without a
 *             physical hit.</li>
 *       </ul>
 *   </li>
 * </ol>
 *
 * <h2>Loop contract</h2>
 * Each cycle, in this order (do not reorder):
 * <ol>
 *   <li>Read Pinpoint linear acceleration (inches/s² → m/s²) and pose.</li>
 *   <li>{@code impactDetector.update(ax, ay)}</li>
 *   <li>{@code planner.update()}</li>
 *   <li>{@code follower.update()}</li>
 * </ol>
 *
 * <h2>Tuning</h2>
 * Bump {@link ImpactDetector} G-threshold if drivetrain vibration false-triggers.
 * Change {@link AdaptivePathPlanner#setCooldownMs(long)} if recovery paths
 * chatter. Edit the AABB calls in {@link #buildGeofence()} to match the current
 * game’s opponent wing / submersible. Clearance is 6 inches.
 */
@TeleOp(name = "Adaptive Pipeline Test", group = "Adaptive")
public class AdaptiveTestOpMode extends OpMode {

    /** Inches per meter — Pedro accel is inches/s²; the detector wants m/s². */
    private static final double INCHES_TO_METERS = 0.0254;

    private static final Pose START_POSE = new Pose(36, 24, 0);
    private static final Pose TARGET_ALPHA = new Pose(108, 36, 0);
    private static final Pose TARGET_BETA = new Pose(108, 108, Math.PI / 2.0);

    private Follower follower;
    private ImpactDetector impactDetector;
    private FoulPreventionBox geofence;
    private AdaptivePathPlanner planner;

    private boolean lastA;
    private boolean lastB;
    private boolean lastX;

    /**
     * Builds the follower (Pinpoint offsets −120 / −150 mm), detector, geofence,
     * and planner. Safe to run while the robot is stationary.
     */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        impactDetector = new ImpactDetector();

        geofence = buildGeofence();

        planner = new AdaptivePathPlanner(follower, impactDetector);
        planner.setFoulPreventionBox(geofence);
        planner.setTargetPose(TARGET_ALPHA);

        telemetry.addLine("Adaptive pipeline initialized.");
        telemetry.addLine("Hold still until PLAY. A=Alpha path  B=Beta goal  X=force replan");
        telemetry.update();
    }

    /**
     * Prints setup reminders while waiting for PLAY.
     */
    @Override
    public void init_loop() {
        telemetry.addLine("Adaptive Pipeline Test");
        telemetry.addData("Pinpoint offsets mm", "strafeX=-120  forwardY=-150");
        telemetry.addData("Start pose in", poseString(START_POSE));
        telemetry.addData("Zones", geofence.getZoneCount());
        telemetry.update();
    }

    /**
     * One non-blocking cycle: sense → detect → replan → follow, then telemetry.
     */
    @Override
    public void loop() {
        handleGamepad();

        Vector accelInches = follower.getAcceleration();
        double ax = accelInches.getXComponent() * INCHES_TO_METERS;
        double ay = accelInches.getYComponent() * INCHES_TO_METERS;

        impactDetector.update(ax, ay);
        planner.update();
        follower.update();

        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();

        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("pose", poseString(pose));
        telemetry.addData("heading deg", Math.toDegrees(pose.getHeading()));
        telemetry.addData("v in/s", String.format("%.1f, %.1f",
                velocity.getXComponent(), velocity.getYComponent()));
        telemetry.addData("a m/s^2", String.format("%.2f, %.2f  |A|=%.2f g",
                ax, ay, impactDetector.getLastHorizontalAccelG()));
        telemetry.addData("impact", impactDetector.isImpactDetected());
        telemetry.addData("impact ms", impactDetector.getImpactDurationMs());
        telemetry.addData("target", poseString(planner.getTargetPose()));
        telemetry.addData("replans", planner.getReplanCount());
        telemetry.addData("cooldown", planner.isCoolingDown());
        telemetry.addData("planner on", planner.isEnabled());
        telemetry.addData("geofence zones", geofence.getZoneCount());
        telemetry.update();
    }

    /**
     * Rising-edge A/B/X on gamepad 1.
     */
    private void handleGamepad() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        if (a && !lastA) {
            planner.setTargetPose(TARGET_ALPHA);
            Path toAlpha = new Path(new BezierLine(follower.getPose(), TARGET_ALPHA));
            toAlpha.setLinearHeadingInterpolation(
                    follower.getPose().getHeading(), TARGET_ALPHA.getHeading());
            follower.followPath(toAlpha, true);
        }
        if (b && !lastB) {
            planner.setTargetPose(TARGET_BETA);
        }
        if (x && !lastX) {
            planner.forceReplan();
        }

        lastA = a;
        lastB = b;
        lastX = x;
    }

    /**
     * Sample INTO-THE-DEEP-style opponent wing and center submersible, 6 in clearance.
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
