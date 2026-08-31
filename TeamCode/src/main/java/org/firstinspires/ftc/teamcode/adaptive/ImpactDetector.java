package org.firstinspires.ftc.teamcode.adaptive;

/**
 * Detects external robot collisions from linear acceleration samples.
 *
 * <p>Intended for high-frequency FTC {@code loop()} use (typically 50–100 Hz). Call
 * {@link #update(double, double)} once per iteration with IMU / Pinpoint linear
 * acceleration in m/s². Detection is based on horizontal magnitude
 *
 * <pre>
 *     A_ext = sqrt(a_x² + a_y²)
 * </pre>
 *
 * compared against a G-force threshold. An impact is flagged only after
 * {@code A_ext} remains above the threshold for a sustained window (default 50 ms),
 * which rejects brief chassis vibration and motor cogging.
 *
 * <p>Once {@link #isImpactDetected()} becomes {@code true}, the flag latches until
 * {@link #resetImpactState()} so a slower planner can consume the event after the
 * spike has already decayed.
 *
 * <p>This class allocates no objects in {@code update} and performs only primitive
 * arithmetic — no blocking I/O, no collections, no garbage-collection pressure.
 */
public class ImpactDetector {

    /** Standard gravity used to convert between g and m/s². */
    public static final double STANDARD_GRAVITY_MPS2 = 9.80665;

    /** Default impact threshold: 1.5 g ≈ 14.71 m/s². */
    public static final double DEFAULT_THRESHOLD_G = 1.5;

    /** {@link #DEFAULT_THRESHOLD_G} expressed in m/s². */
    public static final double DEFAULT_THRESHOLD_MPS2 =
            DEFAULT_THRESHOLD_G * STANDARD_GRAVITY_MPS2;

    /** Default time {@code A_ext} must stay above threshold before latching (ms). */
    public static final long DEFAULT_SUSTAIN_WINDOW_MS = 50L;

    private static final long NANOS_PER_MILLI = 1_000_000L;

    private double thresholdMps2;
    private long sustainWindowNanos;

    /** Latched impact flag; cleared only by {@link #resetImpactState()}. */
    private boolean impactDetected;

    /** NanoTime when the current contiguous over-threshold interval began; 0 if none. */
    private long exceedanceStartNanos;

    /**
     * Duration of the current (or last latched) over-threshold interval, in milliseconds.
     * Frozen after the spike ends if the impact has already latched.
     */
    private long impactDurationMs;

    /** Most recent {@code A_ext} in m/s² (residual if expected accel was supplied). */
    private double lastHorizontalAccelMps2;

    /**
     * Creates a detector with a 1.5 g threshold and a 50 ms sustain window.
     */
    public ImpactDetector() {
        this(DEFAULT_THRESHOLD_MPS2, DEFAULT_SUSTAIN_WINDOW_MS);
    }

    /**
     * Creates a detector with a custom G-force threshold and the default 50 ms window.
     *
     * @param thresholdG horizontal magnitude threshold in g (must be {@code > 0})
     */
    public ImpactDetector(double thresholdG) {
        this(thresholdG * STANDARD_GRAVITY_MPS2, DEFAULT_SUSTAIN_WINDOW_MS);
    }

    /**
     * Creates a detector with explicit SI threshold and sustain window.
     *
     * @param thresholdMps2   horizontal magnitude threshold in m/s² (must be {@code > 0})
     * @param sustainWindowMs required contiguous over-threshold time in milliseconds (must be {@code > 0})
     */
    public ImpactDetector(double thresholdMps2, long sustainWindowMs) {
        setThresholdMps2(thresholdMps2);
        setSustainWindowMs(sustainWindowMs);
    }

    /**
     * Ingests one linear-acceleration sample in the robot or field XY plane.
     *
     * <p>{@code a_z} is ignored: gravity along vertical would dominate {@code A_ext}
     * and false-trigger on chassis pitch. Units must be m/s² (Pinpoint / IMU linear
     * accel), not g.
     *
     * @param accelXMps2 measured linear acceleration along X, m/s²
     * @param accelYMps2 measured linear acceleration along Y, m/s²
     */
    public void update(double accelXMps2, double accelYMps2) {
        updateInternal(accelXMps2, accelYMps2, 0.0, 0.0, System.nanoTime());
    }

    /**
     * Same as {@link #update(double, double)} but subtracts the path-expected
     * acceleration before computing {@code A_ext}. Use this when commanded
     * trajectory accel would otherwise exceed the G threshold (hard braking).
     *
     * <pre>
     *     A_ext = sqrt((a_x − a_x,expected)² + (a_y − a_y,expected)²)
     * </pre>
     *
     * @param accelXMps2         measured X acceleration, m/s²
     * @param accelYMps2         measured Y acceleration, m/s²
     * @param expectedAccelXMps2 path-follower expected X acceleration, m/s²
     * @param expectedAccelYMps2 path-follower expected Y acceleration, m/s²
     */
    public void update(double accelXMps2, double accelYMps2,
                       double expectedAccelXMps2, double expectedAccelYMps2) {
        updateInternal(accelXMps2, accelYMps2, expectedAccelXMps2, expectedAccelYMps2,
                System.nanoTime());
    }

    /**
     * Same as {@link #update(double, double)} with an injected timestamp for tests
     * or when the caller already sampled {@link System#nanoTime()}.
     *
     * @param accelXMps2      measured X acceleration, m/s²
     * @param accelYMps2      measured Y acceleration, m/s²
     * @param timestampNanos  monotonic timestamp, nanoseconds
     */
    public void update(double accelXMps2, double accelYMps2, long timestampNanos) {
        updateInternal(accelXMps2, accelYMps2, 0.0, 0.0, timestampNanos);
    }

    /**
     * @return {@code true} after {@code A_ext} stayed above the threshold for the
     *         sustain window; remains {@code true} until {@link #resetImpactState()}
     */
    public boolean isImpactDetected() {
        return impactDetected;
    }

    /**
     * Clears the latched impact flag and the current exceedance timer.
     * Call after the adaptive planner has consumed the collision event.
     */
    public void resetImpactState() {
        impactDetected = false;
        exceedanceStartNanos = 0L;
        impactDurationMs = 0L;
        lastHorizontalAccelMps2 = 0.0;
    }

    /**
     * @return milliseconds that {@code A_ext} has been (or, if latched and already
     *         below threshold, was) continuously above the threshold; {@code 0} if
     *         no exceedance is in progress and no impact is latched
     */
    public long getImpactDurationMs() {
        return impactDurationMs;
    }

    /**
     * @return most recent horizontal magnitude {@code A_ext} in m/s²
     */
    public double getLastHorizontalAccelMps2() {
        return lastHorizontalAccelMps2;
    }

    /**
     * @return most recent {@code A_ext} expressed in g
     */
    public double getLastHorizontalAccelG() {
        return lastHorizontalAccelMps2 / STANDARD_GRAVITY_MPS2;
    }

    /**
     * @return current magnitude threshold in m/s²
     */
    public double getThresholdMps2() {
        return thresholdMps2;
    }

    /**
     * @return current magnitude threshold in g
     */
    public double getThresholdG() {
        return thresholdMps2 / STANDARD_GRAVITY_MPS2;
    }

    /**
     * Sets the horizontal magnitude threshold in m/s².
     *
     * @param thresholdMps2 must be {@code > 0}
     */
    public void setThresholdMps2(double thresholdMps2) {
        if (thresholdMps2 <= 0.0) {
            throw new IllegalArgumentException("thresholdMps2 must be > 0");
        }
        this.thresholdMps2 = thresholdMps2;
    }

    /**
     * Sets the horizontal magnitude threshold in g (converted with standard gravity).
     *
     * @param thresholdG must be {@code > 0}
     */
    public void setThresholdG(double thresholdG) {
        setThresholdMps2(thresholdG * STANDARD_GRAVITY_MPS2);
    }

    /**
     * @return required contiguous over-threshold time, in milliseconds
     */
    public long getSustainWindowMs() {
        return sustainWindowNanos / NANOS_PER_MILLI;
    }

    /**
     * Sets how long {@code A_ext} must stay above the threshold before latching.
     *
     * @param sustainWindowMs must be {@code > 0}
     */
    public void setSustainWindowMs(long sustainWindowMs) {
        if (sustainWindowMs <= 0L) {
            throw new IllegalArgumentException("sustainWindowMs must be > 0");
        }
        this.sustainWindowNanos = sustainWindowMs * NANOS_PER_MILLI;
    }

    /**
     * Residual horizontal magnitude, then sustain-window / latch logic.
     * No object allocation; safe at OpMode loop rates.
     */
    private void updateInternal(double accelXMps2, double accelYMps2,
                                double expectedAccelXMps2, double expectedAccelYMps2,
                                long timestampNanos) {
        double residualX = accelXMps2 - expectedAccelXMps2;
        double residualY = accelYMps2 - expectedAccelYMps2;
        lastHorizontalAccelMps2 = Math.hypot(residualX, residualY);

        if (lastHorizontalAccelMps2 > thresholdMps2) {
            if (exceedanceStartNanos == 0L) {
                exceedanceStartNanos = timestampNanos;
            }
            long elapsedNanos = timestampNanos - exceedanceStartNanos;
            if (elapsedNanos < 0L) {
                // Clock went backwards (rare); restart the window.
                exceedanceStartNanos = timestampNanos;
                elapsedNanos = 0L;
            }
            impactDurationMs = elapsedNanos / NANOS_PER_MILLI;
            if (elapsedNanos >= sustainWindowNanos) {
                impactDetected = true;
            }
        } else if (!impactDetected) {
            exceedanceStartNanos = 0L;
            impactDurationMs = 0L;
        } else {
            // Impact already latched: freeze duration, wait for resetImpactState().
            exceedanceStartNanos = 0L;
        }
    }
}
