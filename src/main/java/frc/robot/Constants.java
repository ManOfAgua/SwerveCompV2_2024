package frc.robot;

import com.ctre.phoenix6.configs.Slot1Configs;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.FieldConstants;

public class Constants {

        public static final class IDConstants {
                public static final int gyro = 17;
        }

        public static final class ControllerConstants { // for playstation 5 controller
                public static final int driver = 0,
                                operator = 1,
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

        public static final class IntakeConstants {
                public static final int intakeID = 10;

                public static double intakeSpd = 0.6;
        }

        public static final class ShooterConstants {
                public static final int shooterTopID = 12,
                                shooterBtmID = 50;

                public static double shooterSpd = 0.8,
                                shooterSlwSpd = 0.2,
                                shooterKP = 0.1, shooterKI = 0.0, shooterKD = 0.0;
        }

        public static final class ArmConstants {
                public static final int leftarmID = 8, rightarmID = 9;

                public static double armSpd = 0.40,
                                armKP = 0.03, armKI = 0.002, armKD = 0.003,

                                kArmGearRatio = 36.66, kCountsPerRev = 2048,
                                kArmScaleFactor = (360 / (243.316601563 / (kCountsPerRev * kArmGearRatio)));
        }

        public static final class AngleConstants {
                public static double kP = 0.1,
                                kI = 0.0015,
                                kD = 0.0001; // prev 0.0058
        }

        public static final class VisionConstants {

                public static final boolean USE_VISION = true; // Vision enabled or not

                /*
                 * A note about these transforms: They appear to follow the normal cordinate
                 * system (x is right when pos. and so on).
                 */
                public static final Transform3d leftTransform = new Transform3d(-0.281, 0.291, 0.636,
                                new Rotation3d(Units.degreesToRadians(-2.5), Units.degreesToRadians(-30),
                                                Units.degreesToRadians(-10)));
                public static final Transform3d rightTransform = new Transform3d(-0.281, -0.291, 0.636,
                                new Rotation3d(Units.degreesToRadians(2.5), Units.degreesToRadians(-30),
                                                Units.degreesToRadians(10)));

                public static final String CameraName = "PhotonCamera";

                /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
                public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

                public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
                public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
                public static final double NOISY_DISTANCE_METERS = 2.5;
                public static final double DISTANCE_WEIGHT = 7;
                public static final int TAG_PRESENCE_WEIGHT = 10;

                /**
                 * Standard deviations of model states. Increase these numbers to trust your
                 * model's state estimates less. This
                 * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
                 * meters.
                 */
                public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(),
                                Nat.N1(),
                                // if these numbers are less than one, multiplying will do bad things
                                1, // x
                                1, // y
                                1 * Math.PI // theta
                );

                /**
                 * Standard deviations of the vision measurements. Increase these numbers to
                 * trust global measurements from vision
                 * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
                 * radians.
                 */
                public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(),
                                // if these numbers are less than one, multiplying will do bad things
                                .1, // x
                                .1, // y
                                .1);

                // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose
                // to the opposite alliance
                public static final Pose2d FLIPPING_POSE = new Pose2d(
                                new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth),
                                new Rotation2d(Math.PI));
        }

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(22.75 / 2.0, 22.75 / 2.0),
                        // Front right
                        new Translation2d(22.75 / 2.0, -22.75 / 2.0),
                        // Back left
                        new Translation2d(-22.75 / 2.0, 22.75 / 2.0),
                        // Back right
                        new Translation2d(-22.75 / 2.0, -22.75 / 2.0));
}
