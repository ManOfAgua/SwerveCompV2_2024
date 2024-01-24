package frc.robot.disabled;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;


public class Disable extends Command{
    TalonFX lfDrive = new TalonFX(3);
    TalonFX lfSteer = new TalonFX(2);

    TalonFX lbDrive = new TalonFX(1);
    TalonFX lbSteer = new TalonFX(0);

    TalonFX rfDrive = new TalonFX(4);
    TalonFX rfSteer = new TalonFX(5);

    TalonFX rbDrive = new TalonFX(6);
    TalonFX rbSteer = new TalonFX(7);
    
public Disable (CommandSwerveDrivetrain s_Swerve){
    addRequirements(s_Swerve);
}
@Override
public void execute(){
    lfDrive.setNeutralMode(NeutralModeValue.Coast);
    lfSteer.setNeutralMode(NeutralModeValue.Coast);

    lbDrive.setNeutralMode(NeutralModeValue.Coast);
    lbSteer.setNeutralMode(NeutralModeValue.Coast);

    rfDrive.setNeutralMode(NeutralModeValue.Coast);
    rfSteer.setNeutralMode(NeutralModeValue.Coast);

    rbDrive.setNeutralMode(NeutralModeValue.Coast);
    rbSteer.setNeutralMode(NeutralModeValue.Coast);


}
}
