package org.firstinspires.ftc.teamcode.adaptive;

/**
 * Geofence for dynamic Pedro recovery paths: field walls plus illegal Alliance /
 * Opponent zones.
 *
 * <p>Register zones from {@code init()} (not {@code loop()}). Query methods use
 * primitives and preallocated buffers — no {@code new} after construction.
 *
 * <h2>Setup (Pedro inches, origin at a field corner, axes {@code [0, 144]})</h2>
 * <pre>
 * FoulPreventionBox geofence = new FoulPreventionBox();
 * geofence.setFieldBounds(0.0, 0.0, 144.0, 144.0);
 * geofence.setClearanceInches(6.0); // ~robot half-width
 *
 * geofence.addAabb("opponent", 0.0, 0.0, 24.0, 48.0);
 * geofence.addAabb("partner", 120.0, 96.0, 144.0, 144.0);
 * geofence.addPolygon("trap", new double[]{40, 60, 60}, new double[]{0, 0, 20});
 *
 * planner.setFoulPreventionBox(geofence);
 * </pre>
 *
 * <p>{@link AdaptivePathPlanner} runs {@code P1} through
 * {@link #getAdjustedControlPoint(Point, Point, Point)} and drops the replan if
 * the sampled Bézier still clips a restricted region.
 */
public class FoulPreventionBox {

    /** Pedro Pathing full-field size (inches). */
    public static final double PEDRO_FIELD_INCHES = 144.0;

    /** Default extra push-out past a zone face (inches). */
    public static final double DEFAULT_CLEARANCE_INCHES = 4.0;

    private static final int MAX_ZONES = 12;
    private static final int MAX_POLYGON_VERTS = 16;
    private static final int BEZIER_SAMPLES = 10;
    private static final int TYPE_AABB = 0;
    private static final int TYPE_POLYGON = 1;

    private final RestrictedZone[] zones = new RestrictedZone[MAX_ZONES];
    private int zoneCount;

    private double fieldMinX = 0.0;
    private double fieldMinY = 0.0;
    private double fieldMaxX = PEDRO_FIELD_INCHES;
    private double fieldMaxY = PEDRO_FIELD_INCHES;
    private boolean fieldBoundsEnabled = true;
    private double clearanceInches = DEFAULT_CLEARANCE_INCHES;

    private final Point adjustedOut = new Point();
    private final double[] liangT = new double[2];
    private final double[] pushScratch = new double[2];
    private final double[] detourScratch = new double[2];

    /**
     * Creates an empty geofence with Pedro {@code [0, 144]} field bounds enabled.
     */
    public FoulPreventionBox() {
        for (int i = 0; i < MAX_ZONES; i++) {
            zones[i] = new RestrictedZone();
        }
    }

    /**
     * Sets the legal playing rectangle. Endpoints outside this box are illegal.
     *
     * @param minX lower X (inches)
     * @param minY lower Y (inches)
     * @param maxX upper X (inches)
     * @param maxY upper Y (inches)
     */
    public void setFieldBounds(double minX, double minY, double maxX, double maxY) {
        if (maxX <= minX || maxY <= minY) {
            throw new IllegalArgumentException("field max must be greater than min");
        }
        this.fieldMinX = minX;
        this.fieldMinY = minY;
        this.fieldMaxX = maxX;
        this.fieldMaxY = maxY;
        this.fieldBoundsEnabled = true;
    }

    /**
     * Enables or disables out-of-field rejection. Registered zones still apply.
     *
     * @param fieldBoundsEnabled {@code true} to treat field exterior as illegal
     */
    public void setFieldBoundsEnabled(boolean fieldBoundsEnabled) {
        this.fieldBoundsEnabled = fieldBoundsEnabled;
    }

    /**
     * Sets how far a projected control point is pushed past a zone face, and the
     * inset used when clamping to field walls.
     *
     * @param clearanceInches must be {@code >= 0}
     */
    public void setClearanceInches(double clearanceInches) {
        if (clearanceInches < 0.0) {
            throw new IllegalArgumentException("clearanceInches must be >= 0");
        }
        this.clearanceInches = clearanceInches;
    }

    /**
     * @return current push-out / wall inset, inches
     */
    public double getClearanceInches() {
        return clearanceInches;
    }

    /**
     * Registers an axis-aligned illegal box (opponent zone, alliance-partner
     * area, etc.).
     *
     * @param name telemetry label (may be {@code null})
     * @param minX lower X, inches
     * @param minY lower Y, inches
     * @param maxX upper X, inches
     * @param maxY upper Y, inches
     */
    public void addAabb(String name, double minX, double minY, double maxX, double maxY) {
        RestrictedZone zone = nextFreeZone();
        if (maxX < minX) {
            double tmp = minX;
            minX = maxX;
            maxX = tmp;
        }
        if (maxY < minY) {
            double tmp = minY;
            minY = maxY;
            maxY = tmp;
        }
        zone.type = TYPE_AABB;
        zone.name = name;
        zone.minX = minX;
        zone.minY = minY;
        zone.maxX = maxX;
        zone.maxY = maxY;
        zone.vertexCount = 0;
        zoneCount++;
    }

    /**
     * Registers an illegal polygon. Vertices are copied; do not repeat the first
     * vertex at the end.
     *
     * @param name telemetry label (may be {@code null})
     * @param xs   vertex X coordinates, inches
     * @param ys   vertex Y coordinates, inches
     */
    public void addPolygon(String name, double[] xs, double[] ys) {
        if (xs == null || ys == null || xs.length != ys.length) {
            throw new IllegalArgumentException("polygon xs/ys must be non-null and same length");
        }
        if (xs.length < 3) {
            throw new IllegalArgumentException("polygon needs at least 3 vertices");
        }
        if (xs.length > MAX_POLYGON_VERTS) {
            throw new IllegalArgumentException("polygon exceeds MAX_POLYGON_VERTS=" + MAX_POLYGON_VERTS);
        }
        RestrictedZone zone = nextFreeZone();
        zone.type = TYPE_POLYGON;
        zone.name = name;
        zone.vertexCount = xs.length;
        System.arraycopy(xs, 0, zone.xs, 0, xs.length);
        System.arraycopy(ys, 0, zone.ys, 0, ys.length);
        zone.minX = xs[0];
        zone.maxX = xs[0];
        zone.minY = ys[0];
        zone.maxY = ys[0];
        for (int i = 1; i < xs.length; i++) {
            zone.minX = Math.min(zone.minX, xs[i]);
            zone.maxX = Math.max(zone.maxX, xs[i]);
            zone.minY = Math.min(zone.minY, ys[i]);
            zone.maxY = Math.max(zone.maxY, ys[i]);
        }
        zoneCount++;
    }

    /**
     * Removes all Alliance / Opponent zones. Field bounds are unchanged.
     */
    public void clearZones() {
        zoneCount = 0;
    }

    /**
     * @return number of registered illegal zones
     */
    public int getZoneCount() {
        return zoneCount;
    }

    /**
     * Returns whether the straight segment {@code p1 → p2} clips any illegal
     * region (zone interior/boundary or field exterior).
     *
     * @param p1 segment start
     * @param p2 segment end
     * @return {@code true} if the segment is not fully legal
     */
    public boolean intersectsRestrictedZone(Point p1, Point p2) {
        return intersectsRestrictedZone(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }

    /**
     * Primitive overload so callers with raw coordinates skip Point boxing.
     *
     * @param x1 start X
     * @param y1 start Y
     * @param x2 end X
     * @param y2 end Y
     * @return {@code true} if the segment is not fully legal
     */
    public boolean intersectsRestrictedZone(double x1, double y1, double x2, double y2) {
        if (fieldBoundsEnabled && (!pointInField(x1, y1) || !pointInField(x2, y2))) {
            return true;
        }
        for (int i = 0; i < zoneCount; i++) {
            if (segmentHitsZone(zones[i], x1, y1, x2, y2)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns whether a field point is inside an illegal zone or outside the field.
     *
     * @param x field X
     * @param y field Y
     * @return {@code true} if the point is not a legal interior location
     */
    public boolean containsRestrictedPoint(double x, double y) {
        if (fieldBoundsEnabled && !pointInField(x, y)) {
            return true;
        }
        for (int i = 0; i < zoneCount; i++) {
            if (pointInZone(zones[i], x, y)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Samples the quadratic Bézier {@code P0 → P1 → P2} and reports whether any
     * consecutive sample segment clips a restricted region.
     *
     * @param x0 P0 X
     * @param y0 P0 Y
     * @param x1 P1 X
     * @param y1 P1 Y
     * @param x2 P2 X
     * @param y2 P2 Y
     * @return {@code true} if the sampled curve is illegal
     */
    public boolean quadraticBezierIntersectsRestricted(
            double x0, double y0, double x1, double y1, double x2, double y2) {
        double prevX = x0;
        double prevY = y0;
        for (int s = 1; s <= BEZIER_SAMPLES; s++) {
            double t = (double) s / (double) BEZIER_SAMPLES;
            double omt = 1.0 - t;
            double bx = omt * omt * x0 + 2.0 * omt * t * x1 + t * t * x2;
            double by = omt * omt * y0 + 2.0 * omt * t * y1 + t * t * y2;
            if (intersectsRestrictedZone(prevX, prevY, bx, by)) {
                return true;
            }
            prevX = bx;
            prevY = by;
        }
        return false;
    }

    /**
     * If {@code proposedControlPoint} is inside a restricted zone, or the chords
     * {@code start → proposed} / {@code proposed → target} clip a zone, projects
     * it to the nearest safe coordinate: out through the closest AABB face (plus
     * clearance), or perpendicular to the start–target chord, then clamped to
     * the field inset.
     *
     * <p>The returned instance is reused on the next call; copy {@code x,y}
     * immediately if you need to persist them.
     *
     * @param start                Bézier {@code P0}
     * @param target               Bézier {@code P2}
     * @param proposedControlPoint velocity-aware {@code P1}
     * @return safe control point (possibly unchanged)
     */
    public Point getAdjustedControlPoint(Point start, Point target, Point proposedControlPoint) {
        double x = clampX(proposedControlPoint.getX());
        double y = clampY(proposedControlPoint.getY());
        double sx = start.getX();
        double sy = start.getY();
        double tx = target.getX();
        double ty = target.getY();

        for (int pass = 0; pass < 4; pass++) {
            boolean moved = false;
            for (int i = 0; i < zoneCount; i++) {
                if (pointInZone(zones[i], x, y)) {
                    pushOutOfZone(zones[i], x, y);
                    x = clampX(pushScratch[0]);
                    y = clampY(pushScratch[1]);
                    moved = true;
                }
            }
            if (!moved) {
                break;
            }
        }

        if (intersectsRestrictedZone(sx, sy, x, y) || intersectsRestrictedZone(x, y, tx, ty)) {
            offsetPerpendicular(sx, sy, tx, ty, x, y);
            x = detourScratch[0];
            y = detourScratch[1];
        }

        adjustedOut.set(x, y);
        return adjustedOut;
    }

    private RestrictedZone nextFreeZone() {
        if (zoneCount >= MAX_ZONES) {
            throw new IllegalStateException("exceeded MAX_ZONES=" + MAX_ZONES);
        }
        return zones[zoneCount];
    }

    private boolean pointInField(double x, double y) {
        return x >= fieldMinX && x <= fieldMaxX && y >= fieldMinY && y <= fieldMaxY;
    }

    private double clampX(double x) {
        if (!fieldBoundsEnabled) {
            return x;
        }
        double lo = fieldMinX + clearanceInches;
        double hi = fieldMaxX - clearanceInches;
        if (lo > hi) {
            return 0.5 * (fieldMinX + fieldMaxX);
        }
        if (x < lo) {
            return lo;
        }
        if (x > hi) {
            return hi;
        }
        return x;
    }

    private double clampY(double y) {
        if (!fieldBoundsEnabled) {
            return y;
        }
        double lo = fieldMinY + clearanceInches;
        double hi = fieldMaxY - clearanceInches;
        if (lo > hi) {
            return 0.5 * (fieldMinY + fieldMaxY);
        }
        if (y < lo) {
            return lo;
        }
        if (y > hi) {
            return hi;
        }
        return y;
    }

    private boolean segmentHitsZone(RestrictedZone zone, double x1, double y1, double x2, double y2) {
        if (zone.type == TYPE_AABB) {
            return segmentIntersectsAabb(x1, y1, x2, y2, zone.minX, zone.minY, zone.maxX, zone.maxY);
        }
        if (pointInPolygon(zone, x1, y1) || pointInPolygon(zone, x2, y2)) {
            return true;
        }
        int n = zone.vertexCount;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            if (segmentsIntersect(x1, y1, x2, y2, zone.xs[i], zone.ys[i], zone.xs[j], zone.ys[j])) {
                return true;
            }
        }
        return false;
    }

    private boolean pointInZone(RestrictedZone zone, double x, double y) {
        if (zone.type == TYPE_AABB) {
            return x >= zone.minX && x <= zone.maxX && y >= zone.minY && y <= zone.maxY;
        }
        return pointInPolygon(zone, x, y);
    }

    private boolean pointInPolygon(RestrictedZone zone, double x, double y) {
        boolean inside = false;
        int n = zone.vertexCount;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = zone.xs[i];
            double yi = zone.ys[i];
            double xj = zone.xs[j];
            double yj = zone.ys[j];
            if ((yi > y) != (yj > y)) {
                double denom = yj - yi;
                if (denom != 0.0 && x < (xj - xi) * (y - yi) / denom + xi) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    private boolean segmentIntersectsAabb(
            double x1, double y1, double x2, double y2,
            double minX, double minY, double maxX, double maxY) {
        if (pointInAabb(x1, y1, minX, minY, maxX, maxY)
                || pointInAabb(x2, y2, minX, minY, maxX, maxY)) {
            return true;
        }
        liangT[0] = 0.0;
        liangT[1] = 1.0;
        double dx = x2 - x1;
        double dy = y2 - y1;
        if (!clipEdge(-dx, x1 - minX)) {
            return false;
        }
        if (!clipEdge(dx, maxX - x1)) {
            return false;
        }
        if (!clipEdge(-dy, y1 - minY)) {
            return false;
        }
        if (!clipEdge(dy, maxY - y1)) {
            return false;
        }
        return liangT[0] <= liangT[1];
    }

    private static boolean pointInAabb(
            double x, double y, double minX, double minY, double maxX, double maxY) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    private boolean clipEdge(double p, double q) {
        if (p == 0.0) {
            return q >= 0.0;
        }
        double r = q / p;
        if (p < 0.0) {
            if (r > liangT[1]) {
                return false;
            }
            if (r > liangT[0]) {
                liangT[0] = r;
            }
        } else {
            if (r < liangT[0]) {
                return false;
            }
            if (r < liangT[1]) {
                liangT[1] = r;
            }
        }
        return true;
    }

    private static boolean segmentsIntersect(
            double a1x, double a1y, double a2x, double a2y,
            double b1x, double b1y, double b2x, double b2y) {
        double d1 = cross(b2x - b1x, b2y - b1y, a1x - b1x, a1y - b1y);
        double d2 = cross(b2x - b1x, b2y - b1y, a2x - b1x, a2y - b1y);
        double d3 = cross(a2x - a1x, a2y - a1y, b1x - a1x, b1y - a1y);
        double d4 = cross(a2x - a1x, a2y - a1y, b2x - a1x, b2y - a1y);
        return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
                && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
    }

    private static double cross(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }

    private void pushOutOfZone(RestrictedZone zone, double x, double y) {
        if (zone.type == TYPE_AABB) {
            double dl = x - zone.minX;
            double dr = zone.maxX - x;
            double db = y - zone.minY;
            double dt = zone.maxY - y;
            int edge = 0;
            double best = dl;
            if (dr < best) {
                best = dr;
                edge = 1;
            }
            if (db < best) {
                best = db;
                edge = 2;
            }
            if (dt < best) {
                edge = 3;
            }
            if (edge == 0) {
                pushScratch[0] = zone.minX - clearanceInches;
                pushScratch[1] = y;
            } else if (edge == 1) {
                pushScratch[0] = zone.maxX + clearanceInches;
                pushScratch[1] = y;
            } else if (edge == 2) {
                pushScratch[0] = x;
                pushScratch[1] = zone.minY - clearanceInches;
            } else {
                pushScratch[0] = x;
                pushScratch[1] = zone.maxY + clearanceInches;
            }
            return;
        }
        double cx = 0.5 * (zone.minX + zone.maxX);
        double cy = 0.5 * (zone.minY + zone.maxY);
        double vx = x - cx;
        double vy = y - cy;
        double mag = Math.hypot(vx, vy);
        if (mag < 1e-6) {
            vx = 1.0;
            vy = 0.0;
            mag = 1.0;
        }
        double scale = (0.5 * Math.hypot(zone.maxX - zone.minX, zone.maxY - zone.minY)
                + clearanceInches) / mag;
        pushScratch[0] = cx + vx * scale;
        pushScratch[1] = cy + vy * scale;
    }

    private void offsetPerpendicular(
            double sx, double sy, double tx, double ty, double px, double py) {
        double dx = tx - sx;
        double dy = ty - sy;
        double len = Math.hypot(dx, dy);
        double nx;
        double ny;
        if (len < 1e-6) {
            nx = 1.0;
            ny = 0.0;
        } else {
            nx = -dy / len;
            ny = dx / len;
        }
        double step = Math.max(clearanceInches, 6.0);
        for (int dir = 0; dir < 2; dir++) {
            double sign = dir == 0 ? 1.0 : -1.0;
            for (int k = 1; k <= 6; k++) {
                double ax = clampX(px + nx * sign * step * k);
                double ay = clampY(py + ny * sign * step * k);
                if (!containsRestrictedPoint(ax, ay)
                        && !intersectsRestrictedZone(sx, sy, ax, ay)
                        && !intersectsRestrictedZone(ax, ay, tx, ty)) {
                    detourScratch[0] = ax;
                    detourScratch[1] = ay;
                    return;
                }
            }
        }
        detourScratch[0] = clampX(px);
        detourScratch[1] = clampY(py);
    }

    private static final class RestrictedZone {
        int type;
        String name;
        double minX;
        double minY;
        double maxX;
        double maxY;
        int vertexCount;
        final double[] xs = new double[MAX_POLYGON_VERTS];
        final double[] ys = new double[MAX_POLYGON_VERTS];
    }
}
