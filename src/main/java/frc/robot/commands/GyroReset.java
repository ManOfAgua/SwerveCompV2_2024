// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;

public class GyroReset extends Command {
  /** Creates a new GyroReset. */
  Pigeon2 gyro = new Pigeon2(17);
  public GyroReset() {
    gyro.setYaw(0);
      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyro.setYaw(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}