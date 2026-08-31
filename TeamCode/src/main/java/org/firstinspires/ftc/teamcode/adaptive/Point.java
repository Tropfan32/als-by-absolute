package org.firstinspires.ftc.teamcode.adaptive;

/**
 * Mutable 2D field coordinate (inches unless the caller documents otherwise).
 *
 * <p>Designed for reuse: call {@link #set(double, double)} instead of allocating
 * in an OpMode loop.
 */
public final class Point {

    private double x;
    private double y;

    /**
     * Creates a point at the origin.
     */
    public Point() {
        this(0.0, 0.0);
    }

    /**
     * @param x field X
     * @param y field Y
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Overwrites this instance (no allocation).
     *
     * @param x field X
     * @param y field Y
     * @return {@code this} for chaining
     */
    public Point set(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    /**
     * Copies another point into this instance.
     *
     * @param other source; must not be {@code null}
     * @return {@code this}
     */
    public Point set(Point other) {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    /**
     * @return field X
     */
    public double getX() {
        return x;
    }

    /**
     * @return field Y
     */
    public double getY() {
        return y;
    }

    /**
     * @param x field X
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * @param y field Y
     */
    public void setY(double y) {
        this.y = y;
    }
}
