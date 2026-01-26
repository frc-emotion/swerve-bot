package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the vision subsystem.
 * Contains camera configurations, transforms, and tuning parameters.
 * 
 * Coordinate System (WPILib/PhotonVision convention):
 * - X: Forward (positive = front of robot)
 * - Y: Left (positive = left side of robot)
 * - Z: Up (positive = above robot)
 * - Pitch: Rotation around Y axis (positive = tilt up)
 * - Yaw: Rotation around Z axis (positive = rotate left/CCW)
 * - Roll: Rotation around X axis
 */
public final class VisionConstants {
    
    // ==================
    // CAMERA NAMES
    // ==================
    // Must match exactly what's configured in PhotonVision web UI
    public static final String CAMERA_NAME_RIGHT = "mugilanr";
    public static final String CAMERA_NAME_LEFT = "aaranc";
    
    // ==================
    // FIELD LAYOUT
    // ==================
    // Load the 2026 Rebuilt AprilTag field layout from deploy directory

    public static final AprilTagFieldLayout TAG_LAYOUT;
    static {
        AprilTagFieldLayout layout = null;
        try {
            System.out.println("Loading custom field layout from deploy directory...");
            System.out.println("Deploy directory: " + Filesystem.getDeployDirectory());

            layout = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory() + "/FRC2026_WELDED.json"));
        } catch (IOException e) {
            System.err.println("Failed to load custom field layout, falling back to default REEFSCAPE");
            e.printStackTrace();
            
        }
        TAG_LAYOUT = layout;
    }
    
    // ==================
    // CAMERA TRANSFORMS (Robot to Camera)
    // ==================
    // These define where each camera is mounted relative to the robot center
    // Translation: X (forward), Y (left), Z (up) in meters
    // Rotation: Roll, Pitch, Yaw in radians
    
    /**
     * Right camera (mugilanr) - Front right of robot
     * Position: 11.25" forward, 9.25" right, 8" up
     * Orientation: -30° pitch (tilted up), -40° yaw (angled right), 0° roll
     *
     * Note: Y is positive left in WPILib, so right = negative Y
     * Note: Yaw is positive CCW, so angled right = negative yaw
     * Note: Negative pitch = camera tilted up from horizontal
     */
    public static final Transform3d ROBOT_TO_CAM_RIGHT = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(11.25),   // X: forward
            Units.inchesToMeters(-9.25),  // Y: right (negative = right)
            Units.inchesToMeters(8.0)      // Z: up from floor
        ),
        new Rotation3d(
            0,                              // Roll: 0°
            Units.degreesToRadians(-30),    // Pitch: -30° (tilted up from horizontal)
            Units.degreesToRadians(-40)     // Yaw: -40° (angled toward right)
        )
    );
    
    /**
     * Left camera (aaranc) - Front left of robot
     * Position: 11.25" forward, 9.25" left, 8" up
     * Orientation: -30° pitch (tilted up), +40° yaw (angled left), 0° roll
     *
     * Note: Negative pitch = camera tilted up from horizontal
     */
    public static final Transform3d ROBOT_TO_CAM_LEFT = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(11.25),   // X: forward
            Units.inchesToMeters(9.25),   // Y: left (positive = left)
            Units.inchesToMeters(8.0)      // Z: up from floor
        ),
        new Rotation3d(
            0,                              // Roll: 0°
            Units.degreesToRadians(-30),    // Pitch: -30° (tilted up from horizontal)
            Units.degreesToRadians(40)      // Yaw: +40° (angled toward left)
        )
    );
    
    // ==================
    // STANDARD DEVIATIONS
    // ==================
    // These control how much we trust vision vs odometry
    // Lower values = more trust in that measurement
    // Format: [x, y, theta] in meters and radians
    
    /** Single tag standard deviations (less accurate, higher = less trust) */
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);

    /** Multi-tag standard deviations (more accurate due to multiple references) */
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
    
    // ==================
    // VISION TUNING PARAMETERS
    // ==================
    
    /** Maximum distance (meters) at which we trust vision measurements */
    public static final double MAX_VISION_DISTANCE = 4.0;
    
    /** Maximum pose ambiguity we'll accept (0-1, lower is better) */
    public static final double MAX_AMBIGUITY = 0.2;
    
    /** Minimum number of targets for multi-tag to be considered reliable */
    public static final int MIN_MULTI_TAG_TARGETS = 2;
    
    // ==================
    // AIM AND RANGE PARAMETERS
    // ==================
    
    /** P gain for turning toward a target */
    public static final double VISION_TURN_kP = 8.0;

    /** D gain for turning toward a target (dampens oscillation/jitter) */
    public static final double VISION_TURN_kD = 0.1;

    /** P gain for driving toward/away from a target */
    public static final double VISION_DRIVE_kP = 0.5;
    
    /** P gain for strafing toward a target */
    public static final double VISION_STRAFE_kP = 0.5;
    
    /** Default target distance in meters for auto-range */
    public static final double DEFAULT_TARGET_DISTANCE = 1.0;
    
    /** Angle tolerance for aim commands (degrees) */
    public static final double AIM_TOLERANCE_DEGREES = 2.0;
    
    /** Distance tolerance for range commands (meters) */
    public static final double RANGE_TOLERANCE_METERS = 0.05;
    
    // ==================
    // CAMERA PHYSICAL PROPERTIES
    // ==================
    // Used for distance calculation via trigonometry
    
    /** Height of the right camera lens from the floor in meters */
    public static final double CAM_RIGHT_HEIGHT_METERS = Units.inchesToMeters(8.0);
    
    /** Height of the left camera lens from the floor in meters */
    public static final double CAM_LEFT_HEIGHT_METERS = Units.inchesToMeters(8.0);
    
    /** Pitch angle of cameras from horizontal (negative = tilted up) */
    public static final double CAM_PITCH_RADIANS = Units.degreesToRadians(-30);
    
    // ==================
    // 2025 REEFSCAPE APRILTAG HEIGHTS
    // ==================
    // Heights of AprilTag centers from the floor (from game manual)
    // These are used for trigonometric distance calculations
    
    /** Reef AprilTag height (tags 6-11, 17-22) */
    public static final double REEF_TAG_HEIGHT_METERS = Units.inchesToMeters(12.125);
    
    /** Processor AprilTag height (tags 3, 16) */
    public static final double PROCESSOR_TAG_HEIGHT_METERS = Units.inchesToMeters(47.875);
    
    /** Coral Station AprilTag height (tags 1, 2, 12, 13) */
    public static final double CORAL_STATION_TAG_HEIGHT_METERS = Units.inchesToMeters(53.25);
    
    /** Barge AprilTag height (tags 4, 5, 14, 15) */
    public static final double BARGE_TAG_HEIGHT_METERS = Units.inchesToMeters(73.875);
    
    /**
     * Gets the height of an AprilTag based on its ID for 2025 REEFSCAPE.
     * 
     * @param tagId The fiducial ID of the AprilTag
     * @return Height of the tag center from the floor in meters
     */
    public static double getTagHeight(int tagId) {
        // Reef tags
        if ((tagId >= 6 && tagId <= 11) || (tagId >= 17 && tagId <= 22)) {
            return REEF_TAG_HEIGHT_METERS;
        }
        // Processor tags
        if (tagId == 3 || tagId == 16) {
            return PROCESSOR_TAG_HEIGHT_METERS;
        }
        // Coral Station tags
        if (tagId == 1 || tagId == 2 || tagId == 12 || tagId == 13) {
            return CORAL_STATION_TAG_HEIGHT_METERS;
        }
        // Barge tags
        if (tagId == 4 || tagId == 5 || tagId == 14 || tagId == 15) {
            return BARGE_TAG_HEIGHT_METERS;
        }
        // Default fallback
        return REEF_TAG_HEIGHT_METERS;
    }
    
    private VisionConstants() {
        // Prevent instantiation
    }
}
