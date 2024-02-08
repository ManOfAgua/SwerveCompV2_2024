// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subystems.arm;

public class ArmPIDCommand extends Command {

private final arm armSub;
private final PIDController armPID;
private final double goal;
private boolean done;

  public ArmPIDCommand(double setPoint, arm arm) {
    this.armSub = arm;
    this.goal = setPoint;
    this.armPID = new PIDController(ArmConstants.armKP, ArmConstants.armKI, ArmConstants.armKD);
    
    addRequirements(armSub);
  }

  @Override
  public void initialize() {
    armPID.setSetpoint(goal);
    armPID.reset();
    System.out.println("\n\nArm PID Command Has Started\n\n");
    armPID.setTolerance(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        done = armPID.atSetpoint();
    
    double speed = armPID.calculate(armSub.armTickToDegrees(), goal);
    armSub.move(-speed);

    SmartDashboard.putBoolean("Arm Tolerance Check", armPID.atSetpoint());
    SmartDashboard.putNumber("Arm Tolerance", armPID.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("\n\n\n\n Arm Command Has Finish \n\n\n\n\n");
    armSub.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
