package frc.robot;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.Arrays;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IDConstants;
import frc.robot.Util.SwerveVoltageRequest;
// import frc.robot.subsystems.PhotonRunnable;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private SwerveModule[] m_swerveModules;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    public Pigeon2 gyro;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field2d = new Field2d();
    // private PhotonRunnable photonEstimator = new PhotonRunnable();
    private Supplier<Rotation2d> rotationSupplier;
    private Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private OriginPosition originPosition = kBlueAllianceWallRightSide;
    private boolean sawTag = false;


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configNeutralMode(NeutralModeValue.Coast);
        configurePathPlanner();
        driveLimit();
        azimuthLimit();
        gyro = new Pigeon2(IDConstants.gyro);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configNeutralMode(NeutralModeValue.Coast);
        configurePathPlanner();
        driveLimit();
        azimuthLimit();
        gyro = new Pigeon2(IDConstants.gyro);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(
                    new PIDConstants(5, 0, 0),
                    new PIDConstants(7.7, 0.01, 0.8),  // .0069  //3.1, 0.01, 0.0039
                        GeneratedConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field during
                    // auto only.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red & !DriverStation.isTeleop();
                    }
                    return false;
                },
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);

    }

    public void driveLimit() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        for (var swerveModule : Modules) {
            TalonFX driveMotor = swerveModule.getDriveMotor();
            driveMotor.getConfigurator().refresh(currentLimits);

            currentLimits.SupplyCurrentLimit = 40;
            currentLimits.SupplyCurrentThreshold = 55;
            currentLimits.SupplyTimeThreshold = 0.1;
            currentLimits.SupplyCurrentLimitEnable = true;
            driveMotor.getConfigurator().apply(currentLimits);
        }
    }

    public void azimuthLimit() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        for (var swerveModule : Modules) {
            TalonFX driveMotor = swerveModule.getSteerMotor();
            driveMotor.getConfigurator().refresh(currentLimits);

            currentLimits.SupplyCurrentLimit = 20;
            currentLimits.SupplyCurrentThreshold = 25;
            currentLimits.SupplyTimeThreshold = 0.1;
            currentLimits.SupplyCurrentLimitEnable = true;
            driveMotor.getConfigurator().apply(currentLimits);
        }
    }

    public double getgyroAngle() {
        return gyro.getAngle();
    }

    public double getGyroYaw() {
        return gyro.getYaw().getValueAsDouble();
    }

    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    public double gyroPitch() {
        return gyro.getPitch().getValue();
    }

    public void zeroGyro() {
        gyro.setYaw(0);
        // gyro.setYaw(0, .135);
        System.out.printf("Gyro ZeroD", + gyro.getYaw().getValueAsDouble());
    }

    private boolean m_hasReseeded = false;
    Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);

    public void enableHoldHeading() {
        if (!m_hasReseeded) {
            m_targetAngle = getState().Pose
                    .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset))
                    .getRotation();
            m_hasReseeded = true;
        }
    }

    public void disableHoldHeading() {
        m_hasReseeded = false;
    }

    public Rotation2d getHoldHeading() {
        return m_targetAngle;
    }

    // private SwerveVoltageRequest driveVoltageRequest = new
    // SwerveVoltageRequest(true);

    // private SysIdRoutine m_driveSysIdRoutine =
    // new SysIdRoutine(
    // new SysIdRoutine.Config(null, Volts.of(4), null,
    // ModifiedSignalLogger.logState()),
    // new SysIdRoutine.Mechanism(
    // (Measure<Voltage> volts) ->
    // setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
    // null,
    // this));
    // private SwerveVoltageRequest steerVoltageRequest = new
    // SwerveVoltageRequest(false);

    // private SysIdRoutine m_steerSysIdRoutine =
    // new SysIdRoutine(
    // new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    // new SysIdRoutine.Mechanism(
    // (Measure<Voltage> volts) ->
    // setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
    // null,
    // this));

    // private SysIdRoutine m_slipSysIdRoutine =
    // new SysIdRoutine(
    // new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null,
    // ModifiedSignalLogger.logState()),
    // new SysIdRoutine.Mechanism(
    // (Measure<Voltage> volts) ->
    // setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
    // null,
    // this));

    // public Command runDriveQuasiTest(Direction direction)
    // {
    // return m_driveSysIdRoutine.quasistatic(direction);
    // }

    // public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    // return m_driveSysIdRoutine.dynamic(direction);
    // }

    // public Command runSteerQuasiTest(Direction direction)
    // {
    // return m_steerSysIdRoutine.quasistatic(direction);
    // }

    // public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
    // return m_steerSysIdRoutine.dynamic(direction);
    // }

    // public Command runDriveSlipTest()
    // {
    // return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    // }

    // public double[] stator(){
    // double[] currents = new double[Modules.length];
    // for (int i = 0; i< Modules.length; i++) {
    // currents[i] = Modules[i].getDriveMotor().getStatorCurrent().getValue();
    // }
    // return currents;
    // }
    @Override
    public void periodic() {
        // var visionPose = photonEstimator.grabLatestEstimatedPose();
        // if (visionPose != null) {
        //     // New pose from vision
        //     sawTag = true;
        //     var pose2d = visionPose.estimatedPose.toPose2d();
  
        //     this.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
        // }

        SmartDashboard.putNumber("Gyro Angle", getgyroAngle());
        SmartDashboard.putNumber("X Pose", this.getState().Pose.getX());
        SmartDashboard.putNumber("Y Pose", this.getState().Pose.getY());
        SmartDashboard.putNumber("Rotation Pose", this.getState().Pose.getRotation().getDegrees());
        
        // check if gyro is changing once tele-op is enabled
        SmartDashboard.putNumber("Gyro Yaw", getGyroYaw());




        for (int i = 0; i < Modules.length; i++) {
            SmartDashboard.putNumber("CANCODER ANGLES: " + i,
                    Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
        }
    }
}
