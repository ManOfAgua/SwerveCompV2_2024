package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
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
        // coastMode();
        configurePathPlanner();
        gyro = new Pigeon2(IDConstants.gyro);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        // coastMode();
        configurePathPlanner();
        gyro = new Pigeon2(IDConstants.gyro);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        // zeroGyro();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    private void configurePathPlanner() {
        double driveBaseRadius = 16.086679272;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(0, 0, 0.15),
                                            new PIDConstants(0, 0, 0),
                                            Constants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->{
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
        return gyro.getYaw().getValueAsDouble();
    }

    public double gyroPitch(){
        return gyro.getPitch().getValue();
    }

    public void zeroGyro(){
        gyro.setYaw(0);
        System.out.println("Gyro Zero");
    }

    public double frontleftrotationsToMeters(){
        double motorRotations = flDrive.getPosition().getValueAsDouble();
        double wheelRotations = motorRotations / 6.75;
    
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2));
    
        return -positionMeters;
    }
       public double frontrightrotationsToMeters(){
        double motorRotations = frDrive.getPosition().getValueAsDouble();
        double wheelRotations = motorRotations / 6.75;
    
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2));
    
        return positionMeters;
    }

        public double backleftrotationsToMeters(){
        double motorRotations = blDrive.getPosition().getValueAsDouble();
        double wheelRotations = motorRotations / 6.75;
    
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2));
    
        return -positionMeters;
    }
        public double backrightrotationsToMeters(){
        double motorRotations = brDrive.getPosition().getValueAsDouble();
        double wheelRotations = motorRotations / 6.75;
    
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2));
    
        return -positionMeters;
    }

    public double frontleftMS(){
            double rotations = (flDrive.getVelocity().getValueAsDouble())*(2*Math.PI*2);
            double flspeed = rotations / 6.75;
            return Units.inchesToMeters(flspeed);
        }
      public double frontrightMS(){
            double rotations = (frDrive.getVelocity().getValueAsDouble())*(2*Math.PI*2);
            double frspeed = rotations / 6.75;
            return Units.inchesToMeters(frspeed);
        }
      public double backrightMS(){
            double rotations = (brDrive.getVelocity().getValueAsDouble())*(2*Math.PI*2);
            double brspeed = rotations / 6.75;
            return Units.inchesToMeters(brspeed);
        }
      public double backleftMS(){
            double rotations = (blDrive.getVelocity().getValueAsDouble())*(2*Math.PI*2);
            double blspeed = rotations / 6.75;
            return Units.inchesToMeters(blspeed);
        }
    private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    private SysIdRoutine m_driveSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
                null,
                this));
    private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);
    
    private SysIdRoutine m_steerSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
                null,
                this));
    
    private SysIdRoutine m_slipSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
                null,
                this));
        
        public Command runDriveQuasiTest(Direction direction)
        {
            return m_driveSysIdRoutine.quasistatic(direction);
        }
    
        public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
            return m_driveSysIdRoutine.dynamic(direction);
        }
    
        public Command runSteerQuasiTest(Direction direction)
        {
            return m_steerSysIdRoutine.quasistatic(direction);
        }
    
        public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
            return m_steerSysIdRoutine.dynamic(direction);
        }
    
        public Command runDriveSlipTest()
        {
            return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
        }
        
    // public void coastMode(){
    //     if(DriverStation.isDisabled()){
    //     flDrive.setNeutralMode(NeutralModeValue.Coast);
    //     frDrive.setNeutralMode(NeutralModeValue.Coast);
    //     blDrive.setNeutralMode(NeutralModeValue.Coast);
    //     brDrive.setNeutralMode(NeutralModeValue.Coast);
    //     flSteer.setNeutralMode(NeutralModeValue.Coast);
    //     frSteer.setNeutralMode(NeutralModeValue.Coast);
    //     blSteer.setNeutralMode(NeutralModeValue.Coast);
    //     brSteer.setNeutralMode(NeutralModeValue.Coast);
    //     }
    // }


@Override
public void periodic(){
    SmartDashboard.putNumber("Heading", gyroHeading());
    SmartDashboard.putNumber("Front Left Distance", frontleftrotationsToMeters());
    SmartDashboard.putNumber("Front Left Speed", frontleftMS());
    SmartDashboard.putData(new PowerDistribution());
}

}
