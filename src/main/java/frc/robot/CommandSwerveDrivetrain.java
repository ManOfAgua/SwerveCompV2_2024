package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
        public Pigeon2 gyro;

    TalonFX flSteer = new TalonFX(2);
    TalonFX flDrive = new TalonFX(3);
    TalonFX frSteer = new TalonFX(5);
    TalonFX frDrive = new TalonFX(4);
    TalonFX blSteer = new TalonFX(0);
    TalonFX blDrive = new TalonFX(1);
    TalonFX brSteer = new TalonFX(7);
    TalonFX brDrive = new TalonFX(6);


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        brakeMode();
        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        brakeMode();
        configurePathPlanner();
        gyro = new Pigeon2(17);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        gyro = new Pigeon2(17);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            Constants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->false, // Change this if the path needs to be flipped on red vs blue
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

    public double gyroHeading(){
        return gyro.getYaw().getValue();
    }

    public double gyroPitch(){
        return gyro.getPitch().getValue();
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    
    public void brakeMode(){
        if(DriverStation.isDisabled()){
        flDrive.setNeutralMode(NeutralModeValue.Coast);
        frDrive.setNeutralMode(NeutralModeValue.Coast);
        blDrive.setNeutralMode(NeutralModeValue.Coast);
        brDrive.setNeutralMode(NeutralModeValue.Coast);
        flSteer.setNeutralMode(NeutralModeValue.Coast);
        frSteer.setNeutralMode(NeutralModeValue.Coast);
        blSteer.setNeutralMode(NeutralModeValue.Coast);
        brSteer.setNeutralMode(NeutralModeValue.Coast);
        }
    }

}
