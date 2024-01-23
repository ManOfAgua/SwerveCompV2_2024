package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleBoardConfig {
    SendableChooser<String> autonChooser = new SendableChooser<String>();

    public ShuffleBoardConfig() {
        addWidgets();
    }

    public void addWidgets() {
        autonChooser.addOption("AroundTheWorld", "AroundTheWorld");
                
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }
}