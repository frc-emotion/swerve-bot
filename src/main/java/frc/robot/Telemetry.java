package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    /* Reference to drivetrain for motor telemetry */
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;

    /* Module names for logging */
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

    /* Cached status signals for efficient CAN reads */
    private final StatusSignal<Current>[] m_driveStatorCurrents = new StatusSignal[4];
    private final StatusSignal<Current>[] m_driveSupplyCurrents = new StatusSignal[4];
    private final StatusSignal<Voltage>[] m_driveVoltages = new StatusSignal[4];
    private final StatusSignal<Temperature>[] m_driveTemps = new StatusSignal[4];

    private final StatusSignal<Current>[] m_steerStatorCurrents = new StatusSignal[4];
    private final StatusSignal<Current>[] m_steerSupplyCurrents = new StatusSignal[4];
    private final StatusSignal<Voltage>[] m_steerVoltages = new StatusSignal[4];
    private final StatusSignal<Temperature>[] m_steerTemps = new StatusSignal[4];

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     * @param drivetrain The swerve drivetrain for motor telemetry access
     */
    public Telemetry(double maxSpeed, SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        MaxSpeed = maxSpeed;
        m_drivetrain = drivetrain;
        SignalLogger.start();

        /* Cache status signals for each module's motors */
        for (int i = 0; i < 4; i++) {
            SwerveModule<TalonFX, TalonFX, CANcoder> module = m_drivetrain.getModule(i);
            TalonFX driveMotor = module.getDriveMotor();
            TalonFX steerMotor = module.getSteerMotor();

            // Drive motor signals
            m_driveStatorCurrents[i] = driveMotor.getStatorCurrent();
            m_driveSupplyCurrents[i] = driveMotor.getSupplyCurrent();
            m_driveVoltages[i] = driveMotor.getMotorVoltage();
            m_driveTemps[i] = driveMotor.getDeviceTemp();

            // Steer motor signals
            m_steerStatorCurrents[i] = steerMotor.getStatorCurrent();
            m_steerSupplyCurrents[i] = steerMotor.getSupplyCurrent();
            m_steerVoltages[i] = steerMotor.getMotorVoltage();
            m_steerTemps[i] = steerMotor.getDeviceTemp();

            // Create per-module publishers
            String moduleName = MODULE_NAMES[i];
            NetworkTable moduleTable = motorTable.getSubTable(moduleName);

            driveStatorCurrentPubs[i] = moduleTable.getDoubleTopic("DriveStatorCurrent").publish();
            driveSupplyCurrentPubs[i] = moduleTable.getDoubleTopic("DriveSupplyCurrent").publish();
            driveVoltagePubs[i] = moduleTable.getDoubleTopic("DriveVoltage").publish();
            driveTempPubs[i] = moduleTable.getDoubleTopic("DriveTemp").publish();

            steerStatorCurrentPubs[i] = moduleTable.getDoubleTopic("SteerStatorCurrent").publish();
            steerSupplyCurrentPubs[i] = moduleTable.getDoubleTopic("SteerSupplyCurrent").publish();
            steerVoltagePubs[i] = moduleTable.getDoubleTopic("SteerVoltage").publish();
            steerTempPubs[i] = moduleTable.getDoubleTopic("SteerTemp").publish();
        }

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }

        /* Set up Field2d for pose visualization */
        SmartDashboard.putData("Field", field);

        /* Configure vision pose type for Glass/Shuffleboard */
        visionTypePub.set("Field2d");
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Motor telemetry table */
    private final NetworkTable motorTable = inst.getTable("MotorTelemetry");

    /* Per-module drive motor telemetry publishers */
    private final DoublePublisher[] driveStatorCurrentPubs = new DoublePublisher[4];
    private final DoublePublisher[] driveSupplyCurrentPubs = new DoublePublisher[4];
    private final DoublePublisher[] driveVoltagePubs = new DoublePublisher[4];
    private final DoublePublisher[] driveTempPubs = new DoublePublisher[4];

    /* Per-module steer motor telemetry publishers */
    private final DoublePublisher[] steerStatorCurrentPubs = new DoublePublisher[4];
    private final DoublePublisher[] steerSupplyCurrentPubs = new DoublePublisher[4];
    private final DoublePublisher[] steerVoltagePubs = new DoublePublisher[4];
    private final DoublePublisher[] steerTempPubs = new DoublePublisher[4];

    /* Aggregate telemetry publishers */
    private final DoublePublisher totalDriveCurrentPub = motorTable.getDoubleTopic("TotalDriveCurrent").publish();
    private final DoublePublisher totalSteerCurrentPub = motorTable.getDoubleTopic("TotalSteerCurrent").publish();
    private final DoublePublisher totalCurrentPub = motorTable.getDoubleTopic("TotalCurrent").publish();
    private final DoublePublisher maxDriveTempPub = motorTable.getDoubleTopic("MaxDriveTemp").publish();
    private final DoublePublisher maxSteerTempPub = motorTable.getDoubleTopic("MaxSteerTemp").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();
    
    /* Field2d for visualizing poses on the field */
    private final Field2d field = new Field2d();
    
    /* Vision pose visualization - StructPublisher for AdvantageScope drag-and-drop */
    private final NetworkTable visionTable = inst.getTable("Vision");
    private final StructPublisher<Pose2d> visionRightPose = visionTable.getStructTopic("RightCameraPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> visionLeftPose = visionTable.getStructTopic("LeftCameraPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> currentPose = inst.getTable("Odometry").getStructTopic("CurrentPose", Pose2d.struct).publish();
    
    /* Also keep double arrays for compatibility with other tools */
    private final DoubleArrayPublisher visionRightPub = visionTable.getDoubleArrayTopic("RightCameraArray").publish();
    private final DoubleArrayPublisher visionLeftPub = visionTable.getDoubleArrayTopic("LeftCameraArray").publish();
    private final StringPublisher visionTypePub = visionTable.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];
    private final double[] m_visionRightArray = new double[3];
    private final double[] m_visionLeftArray = new double[3];
    
    /* Track last known poses for continuous publishing */
    private Pose2d m_lastRightPose = new Pose2d();
    private Pose2d m_lastLeftPose = new Pose2d();

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);
        
        /* Update Field2d with odometry pose */
        field.setRobotPose(state.Pose);
        
        /* Publish current pose as Pose2d struct for AdvantageScope */
        currentPose.set(state.Pose);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }

        /* Refresh all motor status signals in a single CAN call for efficiency */
        BaseStatusSignal.refreshAll(
            m_driveStatorCurrents[0], m_driveStatorCurrents[1], m_driveStatorCurrents[2], m_driveStatorCurrents[3],
            m_driveSupplyCurrents[0], m_driveSupplyCurrents[1], m_driveSupplyCurrents[2], m_driveSupplyCurrents[3],
            m_driveVoltages[0], m_driveVoltages[1], m_driveVoltages[2], m_driveVoltages[3],
            m_driveTemps[0], m_driveTemps[1], m_driveTemps[2], m_driveTemps[3],
            m_steerStatorCurrents[0], m_steerStatorCurrents[1], m_steerStatorCurrents[2], m_steerStatorCurrents[3],
            m_steerSupplyCurrents[0], m_steerSupplyCurrents[1], m_steerSupplyCurrents[2], m_steerSupplyCurrents[3],
            m_steerVoltages[0], m_steerVoltages[1], m_steerVoltages[2], m_steerVoltages[3],
            m_steerTemps[0], m_steerTemps[1], m_steerTemps[2], m_steerTemps[3]
        );

        /* Publish motor telemetry for each module and calculate aggregates */
        double totalDriveCurrent = 0;
        double totalSteerCurrent = 0;
        double maxDriveTemp = 0;
        double maxSteerTemp = 0;

        for (int i = 0; i < 4; i++) {
            // Get values from status signals
            double driveStatorCurrent = m_driveStatorCurrents[i].getValueAsDouble();
            double driveSupplyCurrent = m_driveSupplyCurrents[i].getValueAsDouble();
            double driveVoltage = m_driveVoltages[i].getValueAsDouble();
            double driveTemp = m_driveTemps[i].getValueAsDouble();

            double steerStatorCurrent = m_steerStatorCurrents[i].getValueAsDouble();
            double steerSupplyCurrent = m_steerSupplyCurrents[i].getValueAsDouble();
            double steerVoltage = m_steerVoltages[i].getValueAsDouble();
            double steerTemp = m_steerTemps[i].getValueAsDouble();

            // Publish per-module telemetry
            driveStatorCurrentPubs[i].set(driveStatorCurrent);
            driveSupplyCurrentPubs[i].set(driveSupplyCurrent);
            driveVoltagePubs[i].set(driveVoltage);
            driveTempPubs[i].set(driveTemp);

            steerStatorCurrentPubs[i].set(steerStatorCurrent);
            steerSupplyCurrentPubs[i].set(steerSupplyCurrent);
            steerVoltagePubs[i].set(steerVoltage);
            steerTempPubs[i].set(steerTemp);

            // Log to SignalLogger for HOOT files
            String moduleName = MODULE_NAMES[i];
            SignalLogger.writeDouble("MotorTelemetry/" + moduleName + "/DriveStatorCurrent", driveStatorCurrent, "A");
            SignalLogger.writeDouble("MotorTelemetry/" + moduleName + "/DriveSupplyCurrent", driveSupplyCurrent, "A");
            SignalLogger.writeDouble("MotorTelemetry/" + moduleName + "/DriveVoltage", driveVoltage, "V");
            SignalLogger.writeDouble("MotorTelemetry/" + moduleName + "/DriveTemp", driveTemp, "C");
            SignalLogger.writeDouble("MotorTelemetry/" + moduleName + "/SteerStatorCurrent", steerStatorCurrent, "A");
            SignalLogger.writeDouble("MotorTelemetry/" + moduleName + "/SteerSupplyCurrent", steerSupplyCurrent, "A");
            SignalLogger.writeDouble("MotorTelemetry/" + moduleName + "/SteerVoltage", steerVoltage, "V");
            SignalLogger.writeDouble("MotorTelemetry/" + moduleName + "/SteerTemp", steerTemp, "C");

            // Accumulate for aggregates (using supply current for total power draw)
            totalDriveCurrent += driveSupplyCurrent;
            totalSteerCurrent += steerSupplyCurrent;
            maxDriveTemp = Math.max(maxDriveTemp, driveTemp);
            maxSteerTemp = Math.max(maxSteerTemp, steerTemp);
        }

        // Publish aggregate telemetry
        totalDriveCurrentPub.set(totalDriveCurrent);
        totalSteerCurrentPub.set(totalSteerCurrent);
        totalCurrentPub.set(totalDriveCurrent + totalSteerCurrent);
        maxDriveTempPub.set(maxDriveTemp);
        maxSteerTempPub.set(maxSteerTemp);

        // Log aggregates to SignalLogger
        SignalLogger.writeDouble("MotorTelemetry/TotalDriveCurrent", totalDriveCurrent, "A");
        SignalLogger.writeDouble("MotorTelemetry/TotalSteerCurrent", totalSteerCurrent, "A");
        SignalLogger.writeDouble("MotorTelemetry/TotalCurrent", totalDriveCurrent + totalSteerCurrent, "A");
        SignalLogger.writeDouble("MotorTelemetry/MaxDriveTemp", maxDriveTemp, "C");
        SignalLogger.writeDouble("MotorTelemetry/MaxSteerTemp", maxSteerTemp, "C");
    }
    
    /**
     * Updates vision pose estimates on the Field2d visualization.
     * Call this from Robot.java after processing vision measurements.
     * 
     * @param rightCameraPose Pose estimate from right camera (null if no estimate)
     * @param leftCameraPose Pose estimate from left camera (null if no estimate)
     */
    public void updateVisionPoses(Pose2d rightCameraPose, Pose2d leftCameraPose) {
        // Update right camera pose - always publish (use last known or zero)
        if (rightCameraPose != null) {
            m_lastRightPose = rightCameraPose;
            m_visionRightArray[0] = rightCameraPose.getX();
            m_visionRightArray[1] = rightCameraPose.getY();
            m_visionRightArray[2] = rightCameraPose.getRotation().getDegrees();
            
            // Add to Field2d as a separate object
            field.getObject("VisionRight").setPose(rightCameraPose);
        }
        // Always publish Pose2d struct for AdvantageScope (last known pose)
        visionRightPose.set(m_lastRightPose);
        visionRightPub.set(m_visionRightArray);
        
        // Update left camera pose - always publish (use last known or zero)
        if (leftCameraPose != null) {
            m_lastLeftPose = leftCameraPose;
            m_visionLeftArray[0] = leftCameraPose.getX();
            m_visionLeftArray[1] = leftCameraPose.getY();
            m_visionLeftArray[2] = leftCameraPose.getRotation().getDegrees();
            
            // Add to Field2d as a separate object
            field.getObject("VisionLeft").setPose(leftCameraPose);
        }
        // Always publish Pose2d struct for AdvantageScope (last known pose)
        visionLeftPose.set(m_lastLeftPose);
        visionLeftPub.set(m_visionLeftArray);
    }
}
