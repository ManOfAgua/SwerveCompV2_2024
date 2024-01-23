package frc.robot.disabled;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;


public class Disable extends Command{
    TalonFX FL = new TalonFX(6);
    TalonFX FR = new TalonFX(8);
    TalonFX BL = new TalonFX(2);
    TalonFX BR = new TalonFX(4);
    
public Disable (CommandSwerveDrivetrain s_Swerve){
    addRequirements(s_Swerve);
}
@Override
public void execute(){
    FL.setNeutralMode(NeutralModeValue.Coast);
    FR.setNeutralMode(NeutralModeValue.Coast);

    BL.setNeutralMode(NeutralModeValue.Coast);
    BR.setNeutralMode(NeutralModeValue.Coast);
}
}
