package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;

public class Constants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 9.46;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 4;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "rio";
    private static final int kPigeonId = 17;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    /*----------------------------------------Front Left----------------------------------------*/
    private static final int kFrontLeftDriveMotorId = 3;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 14;
    private static final double kFrontLeftEncoderOffset = 0.30810546875;

    private static final double kFrontLeftXPosInches = 11.375;
    private static final double kFrontLeftYPosInches = 11.375;

    /*----------------------------------------Front Right----------------------------------------*/
    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 5;
    private static final int kFrontRightEncoderId = 15;
    private static final double kFrontRightEncoderOffset = -0.330810546875;

    private static final double kFrontRightXPosInches = 11.375;
    private static final double kFrontRightYPosInches = -11.375;

    /*----------------------------------------Back Left----------------------------------------*/
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 0;
    private static final int kBackLeftEncoderId = 13;
    private static final double kBackLeftEncoderOffset = 0.266845703125;

    private static final double kBackLeftXPosInches = -11.375;
    private static final double kBackLeftYPosInches = 11.375;

    /*----------------------------------------Back Right----------------------------------------*/
    private static final int kBackRightDriveMotorId = 6;
    private static final int kBackRightSteerMotorId = 7;
    private static final int kBackRightEncoderId = 16;
    private static final double kBackRightEncoderOffset = -0.188232421875;

    private static final double kBackRightXPosInches = -11.375;
    private static final double kBackRightYPosInches = -11.375;
    
    
    /*----------------------------------------Constants----------------------------------------*/
    public static final class ControllerConstants { //for playstation 5 controller
        public static final int
            driver = 0, 
            operator = 1,
        //buttons (b_)
            b_SQR = 1,
            b_X = 2,
            b_O = 3,
            b_TRI = 4,
            b_L1 = 5, 
            b_R1 = 6, 
            b_L2 = 7,
            b_R2 = 8,
            b_PIC = 9, 
            b_MEN = 10,
            b_LJOY = 11, 
            b_RJOY = 12,
            b_LOG = 13, 
            b_PAD = 14, 
            b_MIC = 15;
}

public static final class IntakeConstants{ // TODO: Go over motor constants
    public static final int
        intakeID = 10;

    public static double 
        intakeMotorSpd = 1;
        }

public static final class ShooterConstants{
    public static final int
        shooterTopID = 12,
        shooterBtmID = 11;

    public static double 
        shooterSpeed =1,
    shooterKP = 0.1, shooterKI = 0.0, shooterKD = 0.0;
}

public static final class ArmConstants{
    public static final int 
        armID = 8;

    public static double
    armSpeed = 0.5,

    armKP = 0.1, armKI = 0.1, armKD = 0.0,
    angleKP = 0.1, angleKI = 0.1, angleKD = 0.0;
}

public static final class CascadeConstants{
    public static final int 
        cascadeID = 9;
}
    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}
