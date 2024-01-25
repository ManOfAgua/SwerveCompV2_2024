// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subystems.arm;
import frc.robot.subystems.shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterSeqCommand extends SequentialCommandGroup {
  /** Creates a new ShooterSeqCommand. */
  public ShooterSeqCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //subs
    var arm = new arm();
    var shooter = new shooter();

    //commands
    var armCMD = new ArmPIDCommand(0, arm);
    var shootCMD = new ShooterCommand(Constants.ShooterConstants.shooterSpeed, shooter);
    addCommands(armCMD.withTimeout(3)
    
    
    );
  }
}
