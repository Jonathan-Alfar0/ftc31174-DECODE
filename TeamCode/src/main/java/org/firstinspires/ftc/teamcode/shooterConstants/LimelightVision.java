package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightVision {

    private static final double METERS_TO_INCHES = 39.3701;
    private final Limelight3A limelight;

    /**
     * Master enable/disable for vision-based odometry correction.
     *
     * When false, hasPose() always returns false regardless of whether
     * the Limelight actually has a valid target. This prevents the
     * odometry from being updated by vision at unwanted times.
     *
     * Controlled externally (e.g., by a button in the OpMode that also
     * zeros the turret before allowing a correction cycle).
     */
    private boolean visionEnabled = false;

    public LimelightVision(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // ==================== ENABLE / DISABLE ====================

    /**
     * Enable or disable vision-based odometry correction.
     * Only enable when the turret is zeroed — Limelight pose is most
     * accurate when the camera is in its calibrated (forward-facing) position.
     */
    public void setEnabled(boolean enabled) {
        visionEnabled = enabled;
    }

    /** Returns whether vision correction is currently permitted. */
    public boolean isEnabled() {
        return visionEnabled;
    }

    // ==================== POSE ====================

    /**
     * Returns true only if:
     *   1. Vision is enabled (visionEnabled == true), AND
     *   2. The Limelight actually has a valid result with a bot pose.
     *
     * If visionEnabled is false, always returns false — the odometry
     * system should treat this as "no pose available" and skip the update.
     */
    public boolean hasPose() {
        if (!visionEnabled) return false;
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid() && r.getBotpose() != null;
    }

    /**
     * Returns the current field pose from Limelight, converted to
     * Pedro Pathing coordinate space. Call hasPose() first to confirm
     * a valid result exists before calling this.
     */
    public Pose getFieldPose() {
        Pose3D p = limelight.getLatestResult().getBotpose();
        Pose pose = new Pose(
                p.getPosition().x * METERS_TO_INCHES,
                p.getPosition().y * METERS_TO_INCHES,
                Math.toRadians(p.getOrientation().getYaw()));
        return FTCCoordinates.INSTANCE.convertToPedro(pose);
    }

    // ==================== AUTO ALIGN (turret TX correction) ====================

    /** Returns true if the Limelight has a valid target (ignores visionEnabled). */
    public boolean hasTarget() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }

    /** Horizontal offset to target in degrees. */
    public double getTx() {
        return limelight.getLatestResult().getTx();
    }

    /** Vertical offset to target in degrees. */
    public double getTy() {
        return limelight.getLatestResult().getTy();
    }

    /** Horizontal offset to target in radians. */
    public double getYawRadians() {
        return Math.toRadians(limelight.getLatestResult().getTx());
    }

    // ==================== INTERNAL ====================

    public static class Pose2d {
        public final double x, y, heading;
        public Pose2d(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.heading = h;
        }
    }
}