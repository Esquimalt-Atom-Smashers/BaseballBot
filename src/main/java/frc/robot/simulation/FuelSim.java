package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class FuelSim {
    protected static final double PERIOD = 0.02; // sec
    protected static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
    // Room temperature dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
    protected static final double AIR_DENSITY = 1.2041; // kg/m^3
    /** Ball vs floor/walls/hub geometry — lower = less ping-pong on carpet. */
    protected static final double FIELD_COR = 0.4; // coefficient of restitution with the field
    protected static final double FUEL_COR = 0.15; // coefficient of restitution with another fuel
    protected static final double NET_COR = 0.2; // coefficient of restitution with the net
    protected static final double ROBOT_COR = 0.08; // coefficient of restitution with a robot
    /**
     * Fraction of robot surface speed transferred along the collision normal. Previously 1.0, which
     * added full chassis m/s to the ball each hit (non-physical here because {@link Fuel#addImpulse}
     * is a direct Δv), launching fuel across the field.
     */
    protected static final double ROBOT_PUSH_SPEED_TRANSFER = 0.22;
    protected static final double FUEL_RADIUS = 0.075;
    protected static final double FIELD_LENGTH = 16.51;
    protected static final double FIELD_WIDTH = 8.04;
    protected static final double TRENCH_WIDTH = 1.265;
    protected static final double TRENCH_BLOCK_WIDTH = 0.305;
    protected static final double TRENCH_HEIGHT = 0.565;
    protected static final double TRENCH_BAR_HEIGHT = 0.102;
    protected static final double TRENCH_BAR_WIDTH = 0.152;
    protected static final double FRICTION = 0.2; // horizontal vel loss per sec while on ground
    protected static final double kTangentialWallDamping = 0.65;
    protected static final double kHorizontalSleepSpeedMps = 0.08;
    protected static final double FUEL_MASS = 0.448 * 0.45392; // kgs
    /**
     * When FuelSim depenetrates fuel from the Maple chassis, scale of the matching linear impulse on the dyn4j drive
     * body (0 = fuel-only resolution).
     */
    private static final double kMapleFuelReactionCoupling = 0.9;
    /** Caps Δv from one fuel reaction so dense piles cannot spike Maple velocity. */
    private static final double kMapleFuelReactionMaxDeltaVMps = 1.0;
    /**
     * Fraction of chassis speed along the fuel-departure direction removed per reaction (0..1). Stops wedged
     * driving + multi-pass separation from acting like an undamped spring (runaway oscillation).
     */
    private static final double kMapleFuelInwardVelocityDamp = 0.55;
    /** Ignore inward damping below this (m/s) to reduce jitter. */
    private static final double kMapleFuelInwardVelEpsilonMps = 0.03;
    protected static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
    // Drag coefficient of smooth sphere: https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
    protected static final double DRAG_COF = 0.47; // dimensionless
    protected static final double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

    protected static final Translation3d[] FIELD_XZ_LINE_STARTS = {
        new Translation3d(0, 0, 0),
        new Translation3d(3.96, 1.57, 0),
        new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(4.61, 1.57, 0.165),
        new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        new Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
        new Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
        new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    };

    protected static final Translation3d[] FIELD_XZ_LINE_ENDS = {
        new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
        new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
        new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(
                4.61 + TRENCH_BAR_WIDTH / 2, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    };

    /** Internal state is scalar to avoid {@link Translation3d} churn in hot physics loops. */
    protected static class Fuel {
        double px, py, pz;
        double vx, vy, vz;

        Fuel(double px, double py, double pz, double vx, double vy, double vz) {
            this.px = px;
            this.py = py;
            this.pz = pz;
            this.vx = vx;
            this.vy = vy;
            this.vz = vz;
        }

        Fuel(double px, double py, double pz) {
            this(px, py, pz, 0, 0, 0);
        }

        Fuel(Translation3d pos, Translation3d vel) {
            this(pos.getX(), pos.getY(), pos.getZ(), vel.getX(), vel.getY(), vel.getZ());
        }

        Fuel(Translation3d pos) {
            this(pos.getX(), pos.getY(), pos.getZ());
        }

        protected void update(boolean simulateAirResistance, int subticks) {
            double dt = PERIOD / subticks;
            px += vx * dt;
            py += vy * dt;
            pz += vz * dt;
            if (pz > FUEL_RADIUS) {
                double ax = 0;
                double ay = 0;
                double az = GRAVITY.getZ(); // -9.81 m/s²
                if (simulateAirResistance) {
                    double speed = Math.hypot(Math.hypot(vx, vy), vz);
                    if (speed > 1e-6) {
                        double dragScale = -DRAG_FORCE_FACTOR * speed / FUEL_MASS;
                        ax += vx * dragScale;
                        ay += vy * dragScale;
                        az += vz * dragScale;
                    }
                }
                vx += ax * dt;
                vy += ay * dt;
                vz += az * dt;
            }
            if (pz <= FUEL_RADIUS + 0.03) {
                if (Math.abs(vz) < 0.05) {
                    vz = 0;
                }
                double friction = 1 - FRICTION * dt;
                vx *= friction;
                vy *= friction;
                if (Math.hypot(vx, vy) < kHorizontalSleepSpeedMps) {
                    vx = 0;
                    vy = 0;
                }
            }
            handleFieldCollisions(subticks);
        }

        /**
         * Keeps fuel inside field XY. Unconditional clamp fixes tunneling when robot resolution runs after field
         * collision in the same substep, or when velocity no longer points "into" the wall.
         */
        protected void clampFieldEdgesXY() {
            if (px < FUEL_RADIUS) {
                px = FUEL_RADIUS;
                if (vx < 0) {
                    vx += -(1 + FIELD_COR) * vx;
                    vy *= kTangentialWallDamping;
                }
            } else if (px > FIELD_LENGTH - FUEL_RADIUS) {
                px = FIELD_LENGTH - FUEL_RADIUS;
                if (vx > 0) {
                    vx += -(1 + FIELD_COR) * vx;
                    vy *= kTangentialWallDamping;
                }
            }

            if (py < FUEL_RADIUS) {
                py = FUEL_RADIUS;
                if (vy < 0) {
                    vy += -(1 + FIELD_COR) * vy;
                    vx *= kTangentialWallDamping;
                }
            } else if (py > FIELD_WIDTH - FUEL_RADIUS) {
                py = FIELD_WIDTH - FUEL_RADIUS;
                if (vy > 0) {
                    vy += -(1 + FIELD_COR) * vy;
                    vx *= kTangentialWallDamping;
                }
            }
        }

        protected void handleXZLineCollision(Translation3d lineStart, Translation3d lineEnd) {
            double yMin = Math.min(lineStart.getY(), lineEnd.getY());
            double yMax = Math.max(lineStart.getY(), lineEnd.getY());
            if (py < yMin || py > yMax) return;

            double sx = lineStart.getX(), sz = lineStart.getZ();
            double ex = lineEnd.getX(), ez = lineEnd.getZ();
            double lvx = ex - sx, lvz = ez - sz;
            double lineVecSq = lvx * lvx + lvz * lvz;
            if (lineVecSq < 1e-18) return;

            double t = ((px - sx) * lvx + (pz - sz) * lvz) / lineVecSq;
            double projx = sx + t * lvx;
            double projz = sz + t * lvz;
            double len = Math.sqrt(lineVecSq);
            double dps = Math.hypot(projx - sx, projz - sz);
            double dpe = Math.hypot(projx - ex, projz - ez);
            if (dps + dpe > len + 1e-7) return;

            double dist = Math.hypot(px - projx, pz - projz);
            if (dist > FUEL_RADIUS) return;

            double nx = -lvz / len;
            double nz = lvx / len;
            double pen = FUEL_RADIUS - dist;
            px += nx * pen;
            pz += nz * pen;

            double vdotn = vx * nx + vz * nz;
            if (vdotn > 0) return;
            double imp = (1 + FIELD_COR) * vdotn;
            vx -= imp * nx;
            vz -= imp * nz;
        }

        protected void handleFieldCollisions(int subticks) {
            // floor and bumps
            for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
                handleXZLineCollision(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
            }

            clampFieldEdgesXY();

            // hubs
            handleHubCollisions(Hub.BLUE_HUB, subticks);
            handleHubCollisions(Hub.RED_HUB, subticks);

            handleTrenchCollisions();
        }

        protected void handleHubCollisions(Hub hub, int subticks) {
            hub.handleHubInteraction(this, subticks);
            hub.fuelCollideSide(this);

            double netCollision = hub.fuelHitNet(this);
            if (netCollision != 0) {
                px += netCollision;
                vx *= -NET_COR;
                vy *= NET_COR;
            }
        }

        protected void handleTrenchCollisions() {
            fuelCollideRectangle(
                    this,
                    new Translation3d(3.96, TRENCH_WIDTH, 0),
                    new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(3.96, FIELD_WIDTH - 1.57, 0),
                    new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, 0),
                    new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, 0),
                    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
                    new Translation3d(
                            4.61 + TRENCH_BAR_WIDTH / 2,
                            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
                    new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
                    new Translation3d(
                            FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
                    new Translation3d(
                            FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                            FIELD_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
        }

        protected void addImpulse(double ix, double iy, double iz) {
            vx += ix;
            vy += iy;
            vz += iz;
        }
    }

    /** Squared diameter for ball–ball proximity (avoids sqrt in hot loop). */
    private static final double FUEL_CONTACT_DIST_SQ = (FUEL_RADIUS * 2) * (FUEL_RADIUS * 2);

    /**
     * Impulse-based robot resolution plus fuel–fuel can leave overlap; geometric pass + repeats clear it. Low count
     * keeps CPU bounded.
     */
    private static final int kRobotFuelSeparationPasses = 3;

    /** Extra ball–ball resolution after robot pushes (ordering + depenetration). */
    private static final int kFuelFuelPassesAfterRobot = 2;

    /**
     * On-ground, nearly stationary fuel skips pairwise resolution — settled piles were dominating CPU
     * (dense grid cells × 5 substeps × ~400 bodies).
     */
    private static boolean isSleepingForBallBall(Fuel f) {
        if (f.pz > FUEL_RADIUS + 0.06) {
            return false;
        }
        return f.vx * f.vx + f.vy * f.vy + f.vz * f.vz < 0.06 * 0.06;
    } // End isSleepingForBallBall

    /**
     * Push overlapping fuel apart along center line (no velocity impulse). Used for "sleeping" piles that skipped
     * resolution entirely and could clip; also safe to call when only depenetration is needed.
     */
    private static void separateFuelFuelPositions(Fuel a, Fuel b) {
        double nx = a.px - b.px;
        double ny = a.py - b.py;
        double nz = a.pz - b.pz;
        double distSq = nx * nx + ny * ny + nz * nz;
        if (distSq >= FUEL_CONTACT_DIST_SQ) {
            return;
        }
        double distance = Math.sqrt(distSq);
        if (distance < 1e-9) {
            nx = 1;
            ny = 0;
            nz = 0;
            distance = 1;
        } else {
            nx /= distance;
            ny /= distance;
            nz /= distance;
        }
        double intersection = FUEL_RADIUS * 2 - distance;
        if (intersection <= 0) {
            return;
        }
        double halfSep = intersection * 0.5;
        a.px += nx * halfSep;
        a.py += ny * halfSep;
        a.pz += nz * halfSep;
        b.px -= nx * halfSep;
        b.py -= ny * halfSep;
        b.pz -= nz * halfSep;
    } // End separateFuelFuelPositions

    protected static void handleFuelCollision(Fuel a, Fuel b) {
        double nx = a.px - b.px;
        double ny = a.py - b.py;
        double nz = a.pz - b.pz;
        double distSq = nx * nx + ny * ny + nz * nz;
        if (distSq >= FUEL_CONTACT_DIST_SQ) {
            return;
        }
        double distance = Math.sqrt(distSq);
        if (distance < 1e-9) {
            nx = 1;
            ny = 0;
            nz = 0;
            distance = 1;
        } else {
            nx /= distance;
            ny /= distance;
            nz /= distance;
        }
        double rvx = b.vx - a.vx;
        double rvy = b.vy - a.vy;
        double rvz = b.vz - a.vz;
        double dot = rvx * nx + rvy * ny + rvz * nz;
        double impulse = 0.5 * (1 + FUEL_COR) * dot;
        double intersection = FUEL_RADIUS * 2 - distance;
        double halfSep = intersection * 0.5;
        a.px += nx * halfSep;
        a.py += ny * halfSep;
        a.pz += nz * halfSep;
        b.px -= nx * halfSep;
        b.py -= ny * halfSep;
        b.pz -= nz * halfSep;
        a.addImpulse(impulse * nx, impulse * ny, impulse * nz);
        b.addImpulse(-impulse * nx, -impulse * ny, -impulse * nz);
    }

    protected static final double CELL_SIZE = 0.25;
    protected static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
    protected static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

    @SuppressWarnings("unchecked")
    protected final ArrayList<Fuel>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];
    private final ArrayList<ArrayList<Fuel>> activeCells = new ArrayList<>();

    protected void handleFuelCollisions(ArrayList<Fuel> fuels) {
        // Clear grid
        for (ArrayList<Fuel> cell : activeCells) {
            cell.clear();
        }
        activeCells.clear();

        if (fuels.size() < 2) {
            return;
        } // End early exit no pairs

        // Populate grid
        for (Fuel fuel : fuels) {
            int col = (int) (fuel.px / CELL_SIZE);
            int row = (int) (fuel.py / CELL_SIZE);

            if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
                grid[col][row].add(fuel);
                if (grid[col][row].size() == 1) {
                   activeCells.add(grid[col][row]);
                }
            }
        }

        // Check collisions
        for (Fuel fuel : fuels) {
            int col = (int) (fuel.px / CELL_SIZE);
            int row = (int) (fuel.py / CELL_SIZE);

            // Check 3x3 neighbor cells
            for (int i = col - 1; i <= col + 1; i++) {
                for (int j = row - 1; j <= row + 1; j++) {
                    if (i >= 0 && i < GRID_COLS && j >= 0 && j < GRID_ROWS) {
                        for (Fuel other : grid[i][j]) {
                            if (other == fuel) {
                                continue;
                            }
                            double dx = fuel.px - other.px;
                            double dy = fuel.py - other.py;
                            double dz = fuel.pz - other.pz;
                            if (dx * dx + dy * dy + dz * dz >= FUEL_CONTACT_DIST_SQ) {
                                continue;
                            }
                            if (fuel.hashCode() >= other.hashCode()) {
                                continue;
                            }
                            if (isSleepingForBallBall(fuel) && isSleepingForBallBall(other)) {
                                separateFuelFuelPositions(fuel, other);
                            } else {
                                handleFuelCollision(fuel, other);
                            }
                        }
                    }
                }
            }
        }
    }

    protected ArrayList<Fuel> fuels = new ArrayList<>();
    protected boolean running = false;
    protected boolean simulateAirResistance = false;

    /** When true, only fuel on the blue side (x <= field center, left side) is spawned; red side fuel is omitted. */
    protected boolean showHalfFuel = false;

    /**
     * Registered robots for collision, intake, and launch; indices match {@link #launchFuel(int, LinearVelocity, Angle, Angle, Transform3d)}
     * and {@link #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)}.
     */
    protected final ArrayList<RegisteredFuelRobot> registeredRobots = new ArrayList<>();

    protected ArrayList<SimIntake> intakes = new ArrayList<>();

    private static final class RegisteredFuelRobot {
        final double robotWidth;
        final double robotLength;
        final double bumperHeight;
        final Supplier<Pose2d> pose;
        final Supplier<ChassisSpeeds> fieldSpeeds;
        /** When non-null, fuel depenetration applies matching impulses so Maple feels field fuel. */
        final SwerveDriveSimulation mapleDrive;

        RegisteredFuelRobot(
                double robotWidth,
                double robotLength,
                double bumperHeight,
                Supplier<Pose2d> pose,
                Supplier<ChassisSpeeds> fieldSpeeds,
                SwerveDriveSimulation mapleDrive) {
            this.robotWidth = robotWidth;
            this.robotLength = robotLength;
            this.bumperHeight = bumperHeight;
            this.pose = pose;
            this.fieldSpeeds = fieldSpeeds;
            this.mapleDrive = mapleDrive;
        }
    }
    protected int subticks = 2;

    /** Publish fuel poses every N robot periods (full array is heavy on NT + AdvantageKit). */
    private int fuelsLogPeriod = 1;
    private int fuelsLogCounter = 0;

    /**
     * Creates a new instance of FuelSim
     * @param tableKey NetworkTable to log fuel positions to as an array of {@link Translation3d} structs.
     */
    public FuelSim(String tableKey) {
        // Initialize grid
        for (int i = 0; i < GRID_COLS; i++) {
            for (int j = 0; j < GRID_ROWS; j++) {
                grid[i][j] = new ArrayList<Fuel>();
            }
        }

        fuelPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic(tableKey + "/Fuels", Translation3d.struct)
                .publish();
    }

    /**
     * Creates a new instance of FuelSim with log path "/Fuel Simulation"
     */
    public FuelSim() {
        this("/Fuel Simulation");
    }

    /**
     * Clears the field of fuel
     */
    public void clearFuel() {
        fuels.clear();
    }

    /**
     * Sets whether to spawn only blue-side fuel. When true, only fuel with x <= field center (blue
     * / left side) is spawned on the next {@link #spawnStartingFuel()} (or when Reset Fuel is pressed).
     * Does not affect existing fuel.
     */
    public void setShowHalfFuel(boolean showHalfFuel) {
        this.showHalfFuel = showHalfFuel;
    }

    /** Returns true if only blue-side fuel is spawned. */
    public boolean isShowHalfFuel() {
        return showHalfFuel;
    }

    private static final double FIELD_HALF_LENGTH = FIELD_LENGTH / 2;

    /**
     * Adds fuel at the given position. When {@link #showHalfFuel} is true, only adds if the position
     * is on the blue side (x <= field center, left side); red side fuel is skipped.
     */
    private void addFuelIfVisible(Translation3d pos) {
        if (!showHalfFuel || pos.getX() <= FIELD_HALF_LENGTH) {
            fuels.add(new Fuel(pos));
        }
    }

    /**
     * Spawns fuel in the neutral zone and depots. When {@link #showHalfFuel} is true, only fuel on
     * the blue side of the field is spawned.
     */
    public void spawnStartingFuel() {
        // Center fuel
        Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 6; j++) {
                addFuelIfVisible(center.plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0)));
                addFuelIfVisible(center.plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0)));
                addFuelIfVisible(center.plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0)));
                addFuelIfVisible(center.plus(new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0)));
            }
        }

        // Depots (low x = blue side, high x = red side)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                addFuelIfVisible(new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS));
                addFuelIfVisible(new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS));
                addFuelIfVisible(
                        new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS));
                addFuelIfVisible(
                        new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS));
            }
        }

        // DEBUG: Log XZ lines
        // Translation3d[][] lines = new Translation3d[FIELD_XZ_LINE_STARTS.length][2];
        // for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
        //     lines[i][0] = FIELD_XZ_LINE_STARTS[i];
        //     lines[i][1] = FIELD_XZ_LINE_ENDS[i];
        // }

        // Logger.recordOutput("Fuel Simulation/Lines (debug)", lines);
    }

    protected StructArrayPublisher<Translation3d> fuelPublisher;

    /** AdvantageKit log path for fuel positions (RealOutputs/FieldSimulation/Fuels). */
    private static final String LOG_FUELS_KEY = "FieldSimulation/Fuels";

    /**
     * Adds array of `Translation3d`'s to NetworkTables at tableKey + "/Fuels" and to AdvantageKit
     * at RealOutputs/FieldSimulation/Fuels for AdvantageScope visualization.
     */
    public void logFuels() {
        int n = fuels.size();
        Translation3d[] positions = new Translation3d[n];
        for (int i = 0; i < n; i++) {
            Fuel f = fuels.get(i);
            positions[i] = new Translation3d(f.px, f.py, f.pz);
        }
        fuelPublisher.set(positions);
        Logger.recordOutput(LOG_FUELS_KEY, positions);
    } // End logFuels

    private void logFuelsIfDue() {
        fuelsLogCounter++;
        if (fuelsLogCounter % fuelsLogPeriod != 0) {
            return;
        }
        logFuels();
    } // End logFuelsIfDue

    /**
     * Start the simulation. `updateSim` must still be called every loop
     */
    public void start() {
        running = true;
        fuelsLogCounter = 0;
    } // End start

    /**
     * Pause the simulation.
     */
    public void stop() {
        running = false;
    }

    /** Enables accounting for drag force in physics step **/
    public void enableAirResistance() {
        simulateAirResistance = true;
    }

    /**
     * Sets the number of physics iterations per loop (0.02s)
     * @param subticks
     */
    public void setSubticks(int subticks) {
        this.subticks = subticks;
    } // End setSubticks

    /** How often to publish fuel poses (1 = every 20 ms, 3 ≈ 15 Hz). */
    public void setFuelsLogPeriod(int everyNRobotPeriods) {
        this.fuelsLogPeriod = Math.max(1, everyNRobotPeriods);
    } // End setFuelsLogPeriod

    /**
     * Clears all registered robots and intakes, then registers one robot at index 0.
     *
     * @param width robot width (m), left to right in robot frame (Y)
     * @param length robot length (m), front to back in robot frame (X)
     * @param bumperHeight bumper collision height (m)
     * @param poseSupplier field pose of the robot
     * @param fieldSpeedsSupplier field-relative chassis speeds
     */
    public void registerRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        registerRobot(width, length, bumperHeight, poseSupplier, fieldSpeedsSupplier, null);
    }

    /**
     * @param mapleDriveSimulation optional Maple dyn4j body; when non-null, fuel depenetration applies opposing linear
     *     impulses so the drive sim cannot drive through pinned fuel
     */
    public void registerRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier,
            SwerveDriveSimulation mapleDriveSimulation) {
        registeredRobots.clear();
        intakes.clear();
        registeredRobots.add(new RegisteredFuelRobot(
                width, length, bumperHeight, poseSupplier, fieldSpeedsSupplier, mapleDriveSimulation));
    }

    /**
     * @return index of the added robot in {@link #registeredRobots} (0-based)
     */
    public int addRegisteredRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        return addRegisteredRobot(width, length, bumperHeight, poseSupplier, fieldSpeedsSupplier, null);
    }

    /**
     * @param mapleDriveSimulation optional Maple dyn4j body; when non-null, fuel depenetration applies opposing linear
     *     impulses on that chassis
     */
    public int addRegisteredRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier,
            SwerveDriveSimulation mapleDriveSimulation) {
        registeredRobots.add(new RegisteredFuelRobot(
                width, length, bumperHeight, poseSupplier, fieldSpeedsSupplier, mapleDriveSimulation));
        return registeredRobots.size() - 1;
    }

    public void registerRobot(
            Distance width,
            Distance length,
            Distance bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        registerRobot(
                width.in(Meters),
                length.in(Meters),
                bumperHeight.in(Meters),
                poseSupplier,
                fieldSpeedsSupplier,
                null);
    }

    public void registerRobot(
            Distance width,
            Distance length,
            Distance bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier,
            SwerveDriveSimulation mapleDriveSimulation) {
        registerRobot(
                width.in(Meters),
                length.in(Meters),
                bumperHeight.in(Meters),
                poseSupplier,
                fieldSpeedsSupplier,
                mapleDriveSimulation);
    }

    public int addRegisteredRobot(
            Distance width,
            Distance length,
            Distance bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        return addRegisteredRobot(
                width.in(Meters),
                length.in(Meters),
                bumperHeight.in(Meters),
                poseSupplier,
                fieldSpeedsSupplier,
                null);
    }

    public int addRegisteredRobot(
            Distance width,
            Distance length,
            Distance bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier,
            SwerveDriveSimulation mapleDriveSimulation) {
        return addRegisteredRobot(
                width.in(Meters),
                length.in(Meters),
                bumperHeight.in(Meters),
                poseSupplier,
                fieldSpeedsSupplier,
                mapleDriveSimulation);
    }

    /**
     * To be called periodically
     * Will do nothing if sim is not running
     */
    public void updateSim() {
        if (!running) return;

        stepSim();
    }

    /**
     * Run the simulation forward 1 time step (0.02s)
     */
    public void stepSim() {
        for (int i = 0; i < subticks; i++) {
            for (Fuel fuel : fuels) {
                fuel.update(this.simulateAirResistance, this.subticks);
            }

            handleFuelCollisions(fuels);

            if (!registeredRobots.isEmpty()) {
                for (int pass = 0; pass < kRobotFuelSeparationPasses; pass++) {
                    handleRobotCollisions(fuels);
                    for (RegisteredFuelRobot rr : registeredRobots) {
                        Pose2d robot = rr.pose.get();
                        double rx = robot.getX();
                        double ry = robot.getY();
                        double reachSq = robotFieldInfluenceRadiusSq(rr);
                        for (Fuel fuel : fuels) {
                            double dx = fuel.px - rx;
                            double dy = fuel.py - ry;
                            if (dx * dx + dy * dy > reachSq) {
                                continue;
                            }
                            geometricSeparateFuelFromRobot(fuel, rr);
                        }
                    }
                    for (Fuel fuel : fuels) {
                        fuel.clampFieldEdgesXY();
                    }
                }
                for (int fp = 0; fp < kFuelFuelPassesAfterRobot; fp++) {
                    handleFuelCollisions(fuels);
                }
                handleIntakes(fuels);
            } else {
                for (Fuel fuel : fuels) {
                    fuel.clampFieldEdgesXY();
                }
            }
        }

        logFuelsIfDue();
    } // End stepSim

    /**
     * Adds a fuel onto the field
     * @param pos Position to spawn at
     * @param vel Initial velocity vector
     */
    public void spawnFuel(Translation3d pos, Translation3d vel) {
        fuels.add(new Fuel(pos, vel));
    }

    /**
     * Spawns a fuel onto the field with a specified launch velocity and angles, accounting for robot movement.
     * Launch position is robot pose + robotToLaunchPoint (e.g. robotToTurret so trajectory starts at Turret).
     *
     * @param launchVelocity Initial launch velocity
     * @param hoodAngle Hood angle where 0 is launching horizontally and 90 degrees is launching straight up
     * @param turretYaw Aim direction in <i>robot frame</i> (0 = robot +X forward), added to the robot's field heading for
     *     horizontal velocity. Do not use the composed launch-pose yaw (e.g. after {@code robotToLaunchPoint} with a fixed
     *     twist); that would double-count mount offsets.
     * @param robotToLaunchPoint Transform from robot center to launch point (e.g. Turret pivot); used for launch position only
     * @throws IllegalStateException if robot is not registered
     */
    public void launchFuel(
            LinearVelocity launchVelocity,
            Angle hoodAngle,
            Angle turretYaw,
            Transform3d robotToLaunchPoint) {
        launchFuel(0, launchVelocity, hoodAngle, turretYaw, robotToLaunchPoint);
    }

    /**
     * @param robotIndex index into the registered-robot list used for launch pose and field chassis speeds
     */
    public void launchFuel(
            int robotIndex,
            LinearVelocity launchVelocity,
            Angle hoodAngle,
            Angle turretYaw,
            Transform3d robotToLaunchPoint) {
        if (robotIndex < 0 || robotIndex >= registeredRobots.size()) {
            throw new IllegalStateException("Invalid fuel robot index: " + robotIndex);
        }
        RegisteredFuelRobot registeredRobot = registeredRobots.get(robotIndex);

        Pose2d robotPose = registeredRobot.pose.get();
        Pose3d launchPose = new Pose3d(robotPose).plus(robotToLaunchPoint);
        ChassisSpeeds fieldSpeeds = registeredRobot.fieldSpeeds.get();

        double horizontalVel = Math.cos(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
        double verticalVel = Math.sin(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
        double shotYawFieldRad = turretYaw.in(Radians) + robotPose.getRotation().getRadians();
        double xVel = horizontalVel * Math.cos(shotYawFieldRad);
        double yVel = horizontalVel * Math.sin(shotYawFieldRad);

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, verticalVel));
    }

    /**
     * Spawns a fuel using only a height offset from robot center (legacy). Prefer
     * {@link #launchFuel(LinearVelocity, Angle, Angle, Transform3d)} with robotToTurret so the trajectory starts at the Turret.
     */
    public void launchFuel(LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight) {
        launchFuel(
                launchVelocity,
                hoodAngle,
                turretYaw,
                new Transform3d(new Translation3d(0, 0, launchHeight.in(Meters)), Rotation3d.kZero));
    }

    private static void clampFuelXYPositionOnly(Fuel f) {
        f.px = Math.max(FUEL_RADIUS, Math.min(FIELD_LENGTH - FUEL_RADIUS, f.px));
        f.py = Math.max(FUEL_RADIUS, Math.min(FIELD_WIDTH - FUEL_RADIUS, f.py));
    } // End clampFuelXYPositionOnly

    /** True when low fuel center circle overlaps the robot bumper footprint in field XY (ignores Z above bumper). */
    private static boolean fuelOverlapsRobotBumperFootprint(Fuel fuel, RegisteredFuelRobot rr) {
        if (fuel.pz > rr.bumperHeight) {
            return false;
        }
        Pose2d robot = rr.pose.get();
        double halfL = rr.robotLength * 0.5;
        double halfW = rr.robotWidth * 0.5;
        double cx = robot.getX();
        double cy = robot.getY();
        double theta = robot.getRotation().getRadians();
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double odx = fuel.px - cx;
        double ody = fuel.py - cy;
        double relX = odx * cos + ody * sin;
        double relY = -odx * sin + ody * cos;
        double qx = Math.max(-halfL, Math.min(halfL, relX));
        double qy = Math.max(-halfW, Math.min(halfW, relY));
        double ddx = relX - qx;
        double ddy = relY - qy;
        double rSq = FUEL_RADIUS * FUEL_RADIUS;
        return ddx * ddx + ddy * ddy < rSq - 1e-10;
    } // End fuelOverlapsRobotBumperFootprint

    /**
     * Pinned against a perimeter wall, fuel cannot move normal to the wall (real ball "acts like" part of the wall).
     * If geometric separation would leave overlap after clamping XY, slide along the wall to clear the robot.
     */
    private static void slideFuelAlongWallToClearRobot(Fuel fuel, RegisteredFuelRobot rr) {
        final double step = 0.035;
        final int maxSteps = 48;
        double px0 = fuel.px;
        double py0 = fuel.py;
        for (int sign : new int[] {1, -1}) {
            fuel.px = px0;
            fuel.py = py0;
            for (int i = 0; i < maxSteps; i++) {
                fuel.py += sign * step;
                clampFuelXYPositionOnly(fuel);
                if (!fuelOverlapsRobotBumperFootprint(fuel, rr)) {
                    return;
                }
            }
        }
        for (int sign : new int[] {1, -1}) {
            fuel.px = px0;
            fuel.py = py0;
            for (int i = 0; i < maxSteps; i++) {
                fuel.px += sign * step;
                clampFuelXYPositionOnly(fuel);
                if (!fuelOverlapsRobotBumperFootprint(fuel, rr)) {
                    return;
                }
            }
        }
    } // End slideFuelAlongWallToClearRobot

    /**
     * Applies a linear impulse on the Maple drive body opposite to fuel XY depenetration so pinned fuel resists the
     * chassis instead of only moving the FuelSim particle.
     */
    private void applyMapleReactionFromFuelWorldDisplacement(RegisteredFuelRobot rr, double dpx, double dpy) {
        if (rr.mapleDrive == null) {
            return;
        }
        double distSq = dpx * dpx + dpy * dpy;
        if (distSq < 1e-16) {
            return;
        }
        double dist = Math.sqrt(distSq);
        double ux = dpx / dist;
        double uy = dpy / dist;

        double dtSub = PERIOD / Math.max(1, subticks);
        double robotMass = rr.mapleDrive.getMass().getMass();
        double maxMag = robotMass * kMapleFuelReactionMaxDeltaVMps * dtSub;

        // Several separation passes run per substep; full displacement impulse each pass over-counts contact force
        // and feeds energy into the chassis–controller loop.
        double scale = (FUEL_MASS * kMapleFuelReactionCoupling) / dtSub / kRobotFuelSeparationPasses;
        Vector2 impulse = new Vector2(-dpx * scale, -dpy * scale);
        if (impulse.getMagnitudeSquared() > maxMag * maxMag) {
            impulse.setMagnitude(maxMag);
        }
        rr.mapleDrive.applyImpulse(impulse);

        Vector2 v = rr.mapleDrive.getLinearVelocity();
        double vin = v.x * ux + v.y * uy;
        if (vin > kMapleFuelInwardVelEpsilonMps) {
            double dampMag = robotMass * vin * kMapleFuelInwardVelocityDamp;
            if (dampMag > maxMag) {
                dampMag = maxMag;
            }
            rr.mapleDrive.applyImpulse(new Vector2(-ux * dampMag, -uy * dampMag));
        }
    } // End applyMapleReactionFromFuelWorldDisplacement

    /**
     * Hard positional separation: fuel center must stay at least {@link #FUEL_RADIUS} from the robot XY bumper
     * rectangle (circle vs AABB). Fixes overlap left by a single impulse pass, corners, and clamp-vs-robot ordering.
     * When the MTD would leave the field, {@link #clampFuelXYPositionOnly} pins the ball on the wall; we then slide
     * along the wall so the ball stays a solid obstacle (no clipping through perimeter; robot must "go around").
     */
    private void geometricSeparateFuelFromRobot(Fuel fuel, RegisteredFuelRobot rr) {
        if (fuel.pz > rr.bumperHeight) {
            return;
        }

        double px0 = fuel.px;
        double py0 = fuel.py;

        Pose2d robot = rr.pose.get();
        double halfL = rr.robotLength * 0.5;
        double halfW = rr.robotWidth * 0.5;
        double cx = robot.getX();
        double cy = robot.getY();
        double theta = robot.getRotation().getRadians();
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        double odx = fuel.px - cx;
        double ody = fuel.py - cy;
        double relX = odx * cos + ody * sin;
        double relY = -odx * sin + ody * cos;

        double qx = Math.max(-halfL, Math.min(halfL, relX));
        double qy = Math.max(-halfW, Math.min(halfW, relY));
        double ddx = relX - qx;
        double ddy = relY - qy;
        double distSq = ddx * ddx + ddy * ddy;
        double r = FUEL_RADIUS;
        double rSq = r * r;
        if (distSq >= rSq - 1e-14) {
            return;
        }

        if (distSq < 1e-16) {
            double best = Double.MAX_VALUE;
            double ux = 0;
            double uy = 0;
            double mag = halfL + r - relX;
            if (mag >= 0 && mag < best) {
                best = mag;
                ux = 1;
                uy = 0;
            }
            mag = relX + halfL + r;
            if (mag >= 0 && mag < best) {
                best = mag;
                ux = -1;
                uy = 0;
            }
            mag = halfW + r - relY;
            if (mag >= 0 && mag < best) {
                best = mag;
                ux = 0;
                uy = 1;
            }
            mag = relY + halfW + r;
            if (mag >= 0 && mag < best) {
                best = mag;
                ux = 0;
                uy = -1;
            }
            if (best < Double.MAX_VALUE) {
                relX += ux * best;
                relY += uy * best;
            }
        } else {
            double dist = Math.sqrt(distSq);
            double push = r - dist;
            relX += (ddx / dist) * push;
            relY += (ddy / dist) * push;
        }

        fuel.px = cx + relX * cos - relY * sin;
        fuel.py = cy + relX * sin + relY * cos;
        clampFuelXYPositionOnly(fuel);
        if (fuelOverlapsRobotBumperFootprint(fuel, rr)) {
            slideFuelAlongWallToClearRobot(fuel, rr);
        }
        applyMapleReactionFromFuelWorldDisplacement(rr, fuel.px - px0, fuel.py - py0);
    } // End geometricSeparateFuelFromRobot

    protected void handleRobotCollision(
            Fuel fuel, RegisteredFuelRobot rr, Pose2d robot, Translation2d robotVel) {
        if (fuel.pz > rr.bumperHeight) return;

        double robotWidth = rr.robotWidth;
        double robotLength = rr.robotLength;

        double odx = fuel.px - robot.getX();
        double ody = fuel.py - robot.getY();
        double theta = robot.getRotation().getRadians();
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double relX = odx * cos + ody * sin;
        double relY = -odx * sin + ody * cos;

        double distanceToBottom = -FUEL_RADIUS - robotLength / 2 - relX;
        double distanceToTop = -FUEL_RADIUS - robotLength / 2 + relX;
        double distanceToRight = -FUEL_RADIUS - robotWidth / 2 - relY;
        double distanceToLeft = -FUEL_RADIUS - robotWidth / 2 + relY;

        if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0) return;

        double ox;
        double oy;
        if (distanceToBottom >= distanceToTop
                && distanceToBottom >= distanceToRight
                && distanceToBottom >= distanceToLeft) {
            ox = distanceToBottom;
            oy = 0;
        } else if (distanceToTop >= distanceToBottom
                && distanceToTop >= distanceToRight
                && distanceToTop >= distanceToLeft) {
            ox = -distanceToTop;
            oy = 0;
        } else if (distanceToRight >= distanceToBottom
                && distanceToRight >= distanceToTop
                && distanceToRight >= distanceToLeft) {
            ox = 0;
            oy = distanceToRight;
        } else {
            ox = 0;
            oy = -distanceToLeft;
        }

        double fieldDx = ox * cos - oy * sin;
        double fieldDy = ox * sin + oy * cos;
        fuel.px += fieldDx;
        fuel.py += fieldDy;
        applyMapleReactionFromFuelWorldDisplacement(rr, fieldDx, fieldDy);

        double olen = Math.hypot(ox, oy);
        if (olen < 1e-9) return;
        double nrx = ox / olen;
        double nry = oy / olen;
        double nfx = nrx * cos - nry * sin;
        double nfy = nrx * sin + nry * cos;

        double vdotn = fuel.vx * nfx + fuel.vy * nfy;
        if (vdotn < 0) {
            double j = -vdotn * (1 + ROBOT_COR);
            fuel.addImpulse(j * nfx, j * nfy, 0);
        }
        double robotPush = (robotVel.getX() * nfx + robotVel.getY() * nfy) * ROBOT_PUSH_SPEED_TRANSFER;
        if (robotPush > 0) {
            fuel.addImpulse(robotPush * nfx, robotPush * nfy, 0);
        }
    }

    /** Intake reach beyond robot front (+X) for broad-phase fuel checks (10.5 in). */
    private static final double kIntakeBeyondFrontMeters = 10.5 * 0.0254;

    /**
     * Furthest point on bumpers + extended intake from robot center (robot frame), plus fuel radius
     * and slack — balls outside this field-XY circle skip bumper and intake checks.
     */
    private static double robotFieldInfluenceRadiusSq(RegisteredFuelRobot registeredRobot) {
        double halfL = registeredRobot.robotLength * 0.5;
        double halfW = registeredRobot.robotWidth * 0.5;
        double reach = Math.hypot(halfL + kIntakeBeyondFrontMeters, halfW) + FUEL_RADIUS + 0.15;
        return reach * reach;
    } // End robotFieldInfluenceRadiusSq

    protected void handleRobotCollisions(ArrayList<Fuel> fuels) {
        for (RegisteredFuelRobot registeredRobot : registeredRobots) {
            Pose2d robot = registeredRobot.pose.get();
            ChassisSpeeds speeds = registeredRobot.fieldSpeeds.get();
            Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

            double rx = robot.getX();
            double ry = robot.getY();
            double reachSq = robotFieldInfluenceRadiusSq(registeredRobot);

            for (Fuel fuel : fuels) {
                double dx = fuel.px - rx;
                double dy = fuel.py - ry;
                if (dx * dx + dy * dy > reachSq) {
                    continue;
                }
                handleRobotCollision(fuel, registeredRobot, robot, robotVel);
            }
        }
    } // End handleRobotCollisions

    protected void handleIntakes(ArrayList<Fuel> fuels) {
        for (SimIntake intake : intakes) {
            RegisteredFuelRobot registeredRobot = registeredRobots.get(intake.robotIndex);
            Pose2d robot = registeredRobot.pose.get();
            double rx = robot.getX();
            double ry = robot.getY();
            double reachSq = robotFieldInfluenceRadiusSq(registeredRobot);

            for (int i = 0; i < fuels.size(); i++) {
                Fuel fuel = fuels.get(i);
                double dx = fuel.px - rx;
                double dy = fuel.py - ry;
                if (dx * dx + dy * dy > reachSq) {
                    continue;
                }
                if (intake.shouldIntake(fuel, robot, registeredRobot.bumperHeight)) {
                    fuels.remove(i);
                    i--;
                }
            }
        }
    } // End handleIntakes

    protected static void fuelCollideRectangle(Fuel fuel, Translation3d start, Translation3d end) {
        if (fuel.pz > end.getZ() + FUEL_RADIUS || fuel.pz < start.getZ() - FUEL_RADIUS) return;
        double distanceToLeft = start.getX() - FUEL_RADIUS - fuel.px;
        double distanceToRight = fuel.px - end.getX() - FUEL_RADIUS;
        double distanceToTop = fuel.py - end.getY() - FUEL_RADIUS;
        double distanceToBottom = start.getY() - FUEL_RADIUS - fuel.py;

        if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0) return;

        double cx;
        double cy;
        if (fuel.px < start.getX()
                || (distanceToLeft >= distanceToRight
                        && distanceToLeft >= distanceToTop
                        && distanceToLeft >= distanceToBottom)) {
            cx = distanceToLeft;
            cy = 0;
        } else if (fuel.px >= end.getX()
                || (distanceToRight >= distanceToLeft
                        && distanceToRight >= distanceToTop
                        && distanceToRight >= distanceToBottom)) {
            cx = -distanceToRight;
            cy = 0;
        } else if (fuel.py > end.getY()
                || (distanceToTop >= distanceToLeft
                        && distanceToTop >= distanceToRight
                        && distanceToTop >= distanceToBottom)) {
            cx = 0;
            cy = -distanceToTop;
        } else {
            cx = 0;
            cy = distanceToBottom;
        }

        if (cx != 0) {
            fuel.px += cx;
            fuel.vx += -(1 + FIELD_COR) * fuel.vx;
            fuel.vy *= kTangentialWallDamping;
        } else if (cy != 0) {
            fuel.py += cy;
            fuel.vy += -(1 + FIELD_COR) * fuel.vy;
            fuel.vx *= kTangentialWallDamping;
        }
    }

    /**
     * Intake for registered robot index {@code 0}.
     *
     * @see #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)
     */
    public void registerIntake(
            double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake, Runnable intakeCallback) {
        registerIntake(0, xMin, xMax, yMin, yMax, ableToIntake, intakeCallback);
    }

    /**
     * Adds an intake region in robot frame: when {@code ableToIntake} is true and a fuel lies inside the box
     * {@code [xMin, xMax] × [yMin, yMax]} (relative X/Y, meters) below {@code bumperHeight}, that fuel is removed and
     * {@code intakeCallback} runs.
     *
     * @param robotIndex registered robot whose pose defines the volume
     * @param xMin minimum robot-frame X of the region (m)
     * @param xMax maximum robot-frame X of the region (m)
     * @param yMin minimum robot-frame Y of the region (m)
     * @param yMax maximum robot-frame Y of the region (m)
     * @param ableToIntake whether intake removal is allowed this tick
     * @param intakeCallback invoked when a fuel is removed by this intake
     */
    public void registerIntake(
            int robotIndex,
            double xMin,
            double xMax,
            double yMin,
            double yMax,
            BooleanSupplier ableToIntake,
            Runnable intakeCallback) {
        intakes.add(new SimIntake(robotIndex, xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
    }

    /**
     * @see #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake) {
        registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {});
    }

    /**
     * @see #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax, Runnable intakeCallback) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
    }

    /**
     * @see #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {});
    }

    /**
     * @see #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)
     */
    public void registerIntake(
            Distance xMin, Distance xMax, Distance yMin, Distance yMax, BooleanSupplier ableToIntake, Runnable intakeCallback) {
        registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake, intakeCallback);
    }

    /**
     * @see #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)
     */
    public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax, BooleanSupplier ableToIntake) {
        registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake);
    }

    /**
     * @see #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)
     */
    public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax, Runnable intakeCallback) {
        registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), intakeCallback);
    }

    /**
     * @see #registerIntake(int, double, double, double, double, BooleanSupplier, Runnable)
     */
    public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
        registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
    }

    public static class Hub {
        public static final Hub BLUE_HUB =
                new Hub(new Translation2d(4.61, FIELD_WIDTH / 2), new Translation3d(5.3, FIELD_WIDTH / 2, 0.89), 1);
        public static final Hub RED_HUB = new Hub(
                new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
                new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
                -1);

        protected static final double ENTRY_HEIGHT = 1.83;
        protected static final double ENTRY_RADIUS = 0.56;

        protected static final double SIDE = 1.2;

        protected static final double NET_HEIGHT_MAX = 3.057;
        protected static final double NET_HEIGHT_MIN = 1.5;
        protected static final double NET_OFFSET = SIDE / 2 + 0.261;
        protected static final double NET_WIDTH = 1.484;

        protected final Translation2d center;
        protected final Translation3d exit;
        protected final int exitVelXMult;

        protected int score = 0;

        protected Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
            this.center = center;
            this.exit = exit;
            this.exitVelXMult = exitVelXMult;
        }

        protected void handleHubInteraction(Fuel fuel, int subticks) {
            if (didFuelScore(fuel, subticks)) {
                fuel.px = exit.getX();
                fuel.py = exit.getY();
                fuel.pz = exit.getZ();
                fuel.vx = exitVelXMult * (Math.random() + 0.1) * 1.5;
                fuel.vy = Math.random() * 2 - 1;
                fuel.vz = 0;
                score++;
            }
        }

        protected boolean didFuelScore(Fuel fuel, int subticks) {
            double distXY = Math.hypot(fuel.px - center.getX(), fuel.py - center.getY());
            double prevZ = fuel.pz - fuel.vz * (PERIOD / subticks);
            return distXY <= ENTRY_RADIUS && fuel.pz <= ENTRY_HEIGHT && prevZ > ENTRY_HEIGHT;
        }

        /**
         * Reset this hub's score to 0
         */
        public void resetScore() {
            score = 0;
        }

        /**
         * Get the current count of fuel scored in this hub
         * @return
         */
        public int getScore() {
            return score;
        }

        protected void fuelCollideSide(Fuel fuel) {
            fuelCollideRectangle(
                    fuel,
                    new Translation3d(center.getX() - SIDE / 2, center.getY() - SIDE / 2, 0),
                    new Translation3d(center.getX() + SIDE / 2, center.getY() + SIDE / 2, ENTRY_HEIGHT - 0.1));
        }

        protected double fuelHitNet(Fuel fuel) {
            if (fuel.pz > NET_HEIGHT_MAX || fuel.pz < NET_HEIGHT_MIN) return 0;
            if (fuel.py > center.getY() + NET_WIDTH / 2 || fuel.py < center.getY() - NET_WIDTH / 2) return 0;
            if (fuel.px > center.getX() + NET_OFFSET * exitVelXMult) {
                return Math.max(0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.px - FUEL_RADIUS));
            } else {
                return Math.min(0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.px + FUEL_RADIUS));
            }
        }
    }

    protected class SimIntake {
        final int robotIndex;
        double xMin, xMax, yMin, yMax;
        BooleanSupplier ableToIntake;
        Runnable callback;

        protected SimIntake(
                int robotIndex,
                double xMin,
                double xMax,
                double yMin,
                double yMax,
                BooleanSupplier ableToIntake,
                Runnable intakeCallback) {
            this.robotIndex = robotIndex;
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
            this.ableToIntake = ableToIntake;
            this.callback = intakeCallback;
        }

        protected boolean shouldIntake(Fuel fuel, Pose2d robotPose, double bumperHeightM) {
            if (!ableToIntake.getAsBoolean() || fuel.pz > bumperHeightM) return false;

            double odx = fuel.px - robotPose.getX();
            double ody = fuel.py - robotPose.getY();
            double th = robotPose.getRotation().getRadians();
            double c = Math.cos(th);
            double s = Math.sin(th);
            double relX = odx * c + ody * s;
            double relY = -odx * s + ody * c;

            boolean result = relX >= xMin && relX <= xMax && relY >= yMin && relY <= yMax;
            if (result) {
                callback.run();
            }
            return result;
        }
    }
}