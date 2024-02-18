package frc.robot;

import java.util.function.Supplier;

import javax.swing.text.Position;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IDConstants;
import frc.robot.Util.ModifiedSignalLogger;
import frc.robot.Util.SwerveVoltageRequest;




/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
        public Pigeon2 gyro;
    private final Field2d m_Field2d = new Field2d();
    

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        coastMode();
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
        coastMode();
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
        double driveBaseRadius = 16.08;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(5, 0, 0),
                                            new PIDConstants(7.7, .00, .92),
                                            Constants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()-> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field during auto only.
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
    public void driveLimit(){
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        for (var swerveModule : Modules) {
            TalonFX driveMotor = swerveModule.getDriveMotor();
            driveMotor.getConfigurator().refresh(currentLimits);
        
            currentLimits.SupplyCurrentLimit = 50;
            currentLimits.SupplyCurrentThreshold = 60;
            currentLimits.SupplyTimeThreshold = 0.1;
            currentLimits.SupplyCurrentLimitEnable = true;
            driveMotor.getConfigurator().apply(currentLimits);
    }
}

    public void azimuthLimit(){
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
    public double gyroHeading(){
        return gyro.getYaw().getValueAsDouble();
    }

    public double gyroPitch(){
        return gyro.getPitch().getValue();
    }

    public void zeroGyro(){
        gyro.setYaw(0);
        System.out.println("Gyro Zero");
    }
    
    public void coastMode(){
          for (var swerveModule : Modules) {
            TalonFX driveMotor = swerveModule.getSteerMotor();
            driveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
        
    }

    public double flencoderPos(){
        CANcoder fl = new CANcoder(14);
        return fl.getPosition().getValueAsDouble();
    }

    public double frencoderPos(){
        CANcoder fr = new CANcoder(15);
        return fr.getPosition().getValueAsDouble();
    }

    public double blencoderPos(){
        CANcoder bl = new CANcoder(13);
        return bl.getPosition().getValueAsDouble();
    }

    public double brencoderPos(){
        CANcoder br = new CANcoder(16);
        return br.getPosition().getValueAsDouble();
    }


    // private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    // private SysIdRoutine m_driveSysIdRoutine =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(null, Volts.of(4), null, ModifiedSignalLogger.logState()),
    //         new SysIdRoutine.Mechanism(
    //             (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
    //             null,
    //             this));
    // private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);
    
    // private SysIdRoutine m_steerSysIdRoutine =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
    //         new SysIdRoutine.Mechanism(
    //             (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
    //             null,
    //             this));
    
    // private SysIdRoutine m_slipSysIdRoutine =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
    //         new SysIdRoutine.Mechanism(
    //             (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
    //             null,
    //             this));
        
    //     public Command runDriveQuasiTest(Direction direction)
    //     {
    //         return m_driveSysIdRoutine.quasistatic(direction);
    //     }
    
    //     public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    //         return m_driveSysIdRoutine.dynamic(direction);
    //     }
    
    //     public Command runSteerQuasiTest(Direction direction)
    //     {
    //         return m_steerSysIdRoutine.quasistatic(direction);
    //     }
    
    //     public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
    //         return m_steerSysIdRoutine.dynamic(direction);
    //     }
    
    //     public Command runDriveSlipTest()
    //     {
    //         return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    //     }
        

    // public void flEncoder(){
    //     for (var swerveModule : Modules[1]){
    //         CANcoder flCaNcoder = swerveModule.getCANcoder();
    //         flCaNcoder.getPosition().getValueAsDouble();
    //     }
    // }

    // public double[] stator(){
    //    double[] currents = new double[Modules.length];
    //    for (int i = 0; i< Modules.length; i++) {
    //     currents[i] = Modules[i].getDriveMotor().getStatorCurrent().getValue();
    //    }
    //    return currents;
    // }

@Override
public void periodic(){
    SmartDashboard.putNumber("Heading", gyroHeading());
    SmartDashboard.putNumber("X Pose", this.getState().Pose.getX());
    SmartDashboard.putNumber("Y Pose", this.getState().Pose.getY());
    SmartDashboard.putNumber("Rotation Pose", this.getState().Pose.getRotation().getDegrees());

    SmartDashboard.putNumber("fr coder", frencoderPos());
    SmartDashboard.putNumber("fl coder", flencoderPos());
    SmartDashboard.putNumber("bl coder", blencoderPos());
    SmartDashboard.putNumber("br coder", brencoderPos());

    // SmartDashboard.putNumberArray("Stator Current", stator());
}

}
