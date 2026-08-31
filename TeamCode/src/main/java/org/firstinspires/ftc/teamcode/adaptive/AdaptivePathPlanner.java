package org.firstinspires.ftc.teamcode.adaptive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;

/**
 * Bridges {@link ImpactDetector}, Pedro Pathing's {@link Follower}, and a recovery
 * destination ({@code targetPose}).
 *
 * <p>Recommended OpMode loop: feed the detector from Pinpoint/IMU, call
 * {@link #update()}, then {@code follower.update()}. Idle {@code update()}
 * calls allocate nothing.
 *
 * <h2>Quadratic Bézier injection</h2>
 * When an impact is latched and cooldown has elapsed:
 * <ol>
 *   <li>{@code P0} = live displaced pose from {@link Follower#getPose()}.</li>
 *   <li>{@code P1} = control point along the current linear velocity vector,
 *       or the heading vector if the robot is stopped, so the start tangent
 *       stays continuous (no commanded stop / mechanical jerk).</li>
 *   <li>{@code P2} = {@code targetPose}.</li>
 *   <li>Heading interpolates linearly from the captured start heading to the
 *       target heading.</li>
 *   <li>{@code follower.followPath(path, holdEnd)} swaps the trajectory
 *       immediately (non-blocking). Drive continues on the next
 *       {@code follower.update()}.</li>
 *   <li>{@link ImpactDetector#resetImpactState()} consumes the event so one
 *       collision yields one replan.</li>
 * </ol>
 *
 * <p>If a {@link FoulPreventionBox} is attached, {@code P1} is projected out of
 * illegal Alliance / Opponent boxes before injection. The replan is skipped when
 * the sampled curve would still clip a restricted zone.
 *
 * <p>{@link Path} / {@link BezierCurve} are constructed only on an accepted
 * replan. Pedro curve types are immutable and cannot be recycled in place.
 */
public class AdaptivePathPlanner {

    /** Default ignore-window after a successful replan, in milliseconds. */
    public static final long DEFAULT_COOLDOWN_MS = 300L;

    /**
     * Interior control-point distance as a fraction of remaining path length.
     * 0.35 keeps the curve smooth without overshooting the goal.
     */
    public static final double DEFAULT_CONTROL_POINT_FRACTION = 0.35;

    /** Skip replanning when already this close to the target (inches). */
    public static final double DEFAULT_MIN_REPLAN_DISTANCE_INCHES = 1.0;

    /** Treat velocity below this (inches/s) as stopped; fall back to heading. */
    public static final double DEFAULT_VELOCITY_TANGENT_EPS = 0.5;

    private static final long NANOS_PER_MILLI = 1_000_000L;

    private final Follower follower;
    private final ImpactDetector impactDetector;

    /** Goal pose for recovery Béziers; {@code null} disables replanning. */
    private Pose targetPose;

    private long cooldownNanos;
    private long cooldownUntilNanos;
    private boolean enabled = true;
    private boolean holdEnd = true;
    private double controlPointFraction;
    private double minReplanDistanceInches;
    private double velocityTangentEps;

    private int replanCount;
    private Path lastInjectedPath;

    private FoulPreventionBox foulPrevention;
    private final Point geofenceStart = new Point();
    private final Point geofenceTarget = new Point();
    private final Point geofenceControl = new Point();

    /**
     * Creates a planner with a 300 ms post-replan cooldown.
     *
     * @param follower       Pedro Pathing follower (live pose + path execution)
     * @param impactDetector latched collision detector feeding this planner
     */
    public AdaptivePathPlanner(Follower follower, ImpactDetector impactDetector) {
        this(follower, impactDetector, DEFAULT_COOLDOWN_MS);
    }

    /**
     * Creates a planner with an explicit cooldown window.
     *
     * @param follower       Pedro Pathing follower
     * @param impactDetector latched collision detector
     * @param cooldownMs     post-replan ignore window in milliseconds
     */
    public AdaptivePathPlanner(Follower follower, ImpactDetector impactDetector, long cooldownMs) {
        if (follower == null) {
            throw new IllegalArgumentException("follower must not be null");
        }
        if (impactDetector == null) {
            throw new IllegalArgumentException("impactDetector must not be null");
        }
        this.follower = follower;
        this.impactDetector = impactDetector;
        this.controlPointFraction = DEFAULT_CONTROL_POINT_FRACTION;
        this.minReplanDistanceInches = DEFAULT_MIN_REPLAN_DISTANCE_INCHES;
        this.velocityTangentEps = DEFAULT_VELOCITY_TANGENT_EPS;
        setCooldownMs(cooldownMs);
    }

    /**
     * Consumes a latched impact and, if allowed, injects a recovery path.
     *
     * <p>Zero-allocation when no impact is latched. If the planner is disabled,
     * {@code targetPose} is {@code null}, or cooldown is active, the latched
     * impact is cleared via {@link ImpactDetector#resetImpactState()} and
     * dropped so a stale event cannot fire the moment planning is re-enabled.
     *
     * @return {@code true} if a new path was injected this call
     */
    public boolean update() {
        if (!impactDetector.isImpactDetected()) {
            return false;
        }

        if (!enabled || targetPose == null) {
            consumeLatchedImpact();
            return false;
        }

        long now = System.nanoTime();
        if (now < cooldownUntilNanos) {
            consumeLatchedImpact();
            return false;
        }

        boolean injected = injectRecoveryPath();
        consumeLatchedImpact();
        if (injected) {
            cooldownUntilNanos = now + cooldownNanos;
        }
        return injected;
    }

    /**
     * Sets the destination used for every recovery Bézier ({@code P2}).
     *
     * <p>Call this when the autonomous sequence changes goals. Passing
     * {@code null} disables replanning; latched impacts are still consumed in
     * {@link #update()} so they cannot fire later when a target is assigned.
     *
     * @param targetPose goal pose in Pedro field inches / radians, or {@code null}
     */
    public void setTargetPose(Pose targetPose) {
        this.targetPose = targetPose;
    }

    /**
     * Sets the recovery destination from field components (inches, radians).
     *
     * @param x       field X, inches
     * @param y       field Y, inches
     * @param heading target heading, radians
     */
    public void setTargetPose(double x, double y, double heading) {
        this.targetPose = new Pose(x, y, heading);
    }

    /**
     * Returns the current recovery destination.
     *
     * @return goal pose, or {@code null} if unset
     */
    public Pose getTargetPose() {
        return targetPose;
    }

    /**
     * Injects a recovery path from the live pose without waiting for an impact.
     * Intended for tests and manual override. Respects {@link #setEnabled(boolean)}
     * and requires a non-null {@code targetPose}. Does not touch the detector.
     *
     * @return {@code true} if a path was injected
     */
    public boolean forceReplan() {
        if (!enabled || targetPose == null) {
            return false;
        }
        boolean injected = injectRecoveryPath();
        if (injected) {
            cooldownUntilNanos = System.nanoTime() + cooldownNanos;
        }
        return injected;
    }

    /**
     * Returns whether automatic collision replanning is enabled.
     *
     * @return {@code true} if {@link #update()} may inject paths
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Enables or disables automatic replanning.
     *
     * <p>When {@code false}, {@link #update()} still clears impact latches but
     * never calls {@code followPath}, so a hit while disabled cannot instantly
     * replan when the planner is turned back on.
     *
     * @param enabled {@code true} to allow recovery injection
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Returns whether the follower should hold the last point after the
     * recovery path finishes.
     *
     * @return current {@code holdEnd} flag passed to {@code followPath}
     */
    public boolean isHoldEnd() {
        return holdEnd;
    }

    /**
     * Sets Pedro's {@code followPath(path, holdEnd)} flag.
     *
     * @param holdEnd {@code true} (default) holds {@code targetPose} after arrival
     */
    public void setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
    }

    /**
     * Returns the configured post-replan ignore window.
     *
     * @return cooldown duration in milliseconds
     */
    public long getCooldownMs() {
        return cooldownNanos / NANOS_PER_MILLI;
    }

    /**
     * Sets how long extra impacts are consumed and dropped after a replan,
     * preventing trajectory thrashing from a sustained collision.
     *
     * @param cooldownMs ignore window; must be {@code > 0} (default 300 ms)
     */
    public void setCooldownMs(long cooldownMs) {
        if (cooldownMs <= 0L) {
            throw new IllegalArgumentException("cooldownMs must be > 0");
        }
        this.cooldownNanos = cooldownMs * NANOS_PER_MILLI;
    }

    /**
     * Returns whether the planner is still ignoring new impact latches.
     *
     * @return {@code true} while post-replan cooldown is active
     */
    public boolean isCoolingDown() {
        return System.nanoTime() < cooldownUntilNanos;
    }

    /**
     * Returns the fraction of remaining distance used to place {@code P1}.
     *
     * @return control-point fraction in {@code (0, 1)}
     */
    public double getControlPointFraction() {
        return controlPointFraction;
    }

    /**
     * Sets how far along the start tangent {@code P1} is placed, as a fraction
     * of remaining distance to {@code P2}. Typical range 0.2–0.5.
     *
     * @param controlPointFraction must be in {@code (0, 1)}
     */
    public void setControlPointFraction(double controlPointFraction) {
        if (controlPointFraction <= 0.0 || controlPointFraction >= 1.0) {
            throw new IllegalArgumentException("controlPointFraction must be in (0, 1)");
        }
        this.controlPointFraction = controlPointFraction;
    }

    /**
     * Returns the distance below which a replan is skipped because the robot
     * is already at the goal.
     *
     * @return minimum remaining distance in inches
     */
    public double getMinReplanDistanceInches() {
        return minReplanDistanceInches;
    }

    /**
     * Sets the remaining-distance threshold that skips injection when the
     * robot is already at {@code targetPose}.
     *
     * @param minReplanDistanceInches must be {@code >= 0}, inches
     */
    public void setMinReplanDistanceInches(double minReplanDistanceInches) {
        if (minReplanDistanceInches < 0.0) {
            throw new IllegalArgumentException("minReplanDistanceInches must be >= 0");
        }
        this.minReplanDistanceInches = minReplanDistanceInches;
    }

    /**
     * Returns how many recovery paths this instance has injected.
     *
     * @return successful {@code followPath} count
     */
    public int getReplanCount() {
        return replanCount;
    }

    /**
     * Returns the last path handed to the follower.
     *
     * @return last injected {@link Path}, or {@code null} if none yet
     */
    public Path getLastInjectedPath() {
        return lastInjectedPath;
    }

    /**
     * Attaches field / Alliance-zone geofencing used when building recovery
     * Béziers. {@code null} disables geofence checks (control point is used as-is).
     *
     * @param foulPrevention preconfigured {@link FoulPreventionBox}, or {@code null}
     */
    public void setFoulPreventionBox(FoulPreventionBox foulPrevention) {
        this.foulPrevention = foulPrevention;
    }

    /**
     * Returns the geofence used during replans.
     *
     * @return attached {@link FoulPreventionBox}, or {@code null} if unset
     */
    public FoulPreventionBox getFoulPreventionBox() {
        return foulPrevention;
    }

    /**
     * Clears a latched collision without building a path.
     */
    private void consumeLatchedImpact() {
        impactDetector.resetImpactState();
    }

    /**
     * Builds {@code P0 → P1 → P2}, sets linear heading interpolation, and
     * swaps the follower trajectory. Allocates {@link BezierCurve} and
     * {@link Path} only here.
     *
     * @return {@code false} if remaining distance is below the skip threshold
     */
    private boolean injectRecoveryPath() {
        Pose start = follower.getPose();
        double remaining = start.distanceFrom(targetPose);
        if (remaining < minReplanDistanceInches) {
            return false;
        }

        Pose control = buildTangentControlPoint(start, remaining);
        control = applyGeofence(start, control);
        if (control == null) {
            return false;
        }

        Path recovery = new Path(new BezierCurve(start, control, targetPose));
        recovery.setLinearHeadingInterpolation(start.getHeading(), targetPose.getHeading());

        follower.followPath(recovery, holdEnd);
        lastInjectedPath = recovery;
        replanCount++;
        return true;
    }

    /**
     * Places {@code P1} along unit linear velocity, or along heading if speed
     * is below {@link #DEFAULT_VELOCITY_TANGENT_EPS}.
     */
    private Pose buildTangentControlPoint(Pose start, double remainingDistance) {
        Vector velocity = follower.getVelocity();
        double speed = velocity.getMagnitude();

        double tangentX;
        double tangentY;
        if (speed > velocityTangentEps) {
            double inv = 1.0 / speed;
            tangentX = velocity.getXComponent() * inv;
            tangentY = velocity.getYComponent() * inv;
        } else {
            double heading = start.getHeading();
            tangentX = Math.cos(heading);
            tangentY = Math.sin(heading);
        }

        double offset = remainingDistance * controlPointFraction;
        return new Pose(
                start.getX() + tangentX * offset,
                start.getY() + tangentY * offset,
                start.getHeading());
    }

    /**
     * Projects {@code P1} out of restricted zones. Returns {@code null} when even
     * the adjusted curve would clip a foul region (replan is aborted).
     */
    private Pose applyGeofence(Pose start, Pose proposedControl) {
        if (foulPrevention == null) {
            return proposedControl;
        }

        geofenceStart.set(start.getX(), start.getY());
        geofenceTarget.set(targetPose.getX(), targetPose.getY());
        geofenceControl.set(proposedControl.getX(), proposedControl.getY());

        Point adjusted = foulPrevention.getAdjustedControlPoint(
                geofenceStart, geofenceTarget, geofenceControl);
        double ax = adjusted.getX();
        double ay = adjusted.getY();

        if (foulPrevention.quadraticBezierIntersectsRestricted(
                start.getX(), start.getY(),
                ax, ay,
                targetPose.getX(), targetPose.getY())) {
            return null;
        }

        return new Pose(ax, ay, proposedControl.getHeading());
    }
}
