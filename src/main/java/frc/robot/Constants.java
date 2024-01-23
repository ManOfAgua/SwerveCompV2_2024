package frc.robot;

public class Constants {
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

}
