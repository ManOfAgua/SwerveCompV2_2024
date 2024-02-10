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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IDConstants;




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
        double driveBaseRadius = 14;
        gyro = new Pigeon2(IDConstants.gyro);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
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
// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine routine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                flDrive.setVoltage(volts.in(Volts));
                frDrive.setVoltage(volts.in(Volts));

                blDrive.setVoltage(volts.in(Volts));
                brDrive.setVoltage(-volts.in(Volts));

              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("flDrive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            flDrive.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontleftrotationsToMeters(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontleftMS(), MetersPerSecond));
                log.motor("blDrive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            blDrive.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backleftrotationsToMeters(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backleftMS(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("frDrive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frDrive.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontrightrotationsToMeters(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontrightMS(),MetersPerSecond));

                log.motor("brDrive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            brDrive.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backrightrotationsToMeters(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backrightMS(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
      }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
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
}

}
