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
import frc.robot.subystems.intake;

public class IntakePIDCommand extends Command {

private final intake intakeSub;
private final PIDController armPID;
private final double goal;
private boolean done;
private boolean _stop;

  public IntakePIDCommand(double setPoint, intake intake, boolean stop) {
    stop = _stop;
    this.intakeSub = intake;
    this.goal = setPoint;
    this.armPID = new PIDController(ArmConstants.armKP, ArmConstants.armKI, ArmConstants.armKD);
    
    addRequirements(intakeSub);
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
    if(_stop)
    {
      double speed = armPID.calculate(goal);
    intakeSub.move(-speed);
    }
    else{
      intakeSub.move(0);
    }
      
    SmartDashboard.putBoolean("Arm Tolerance Check", armPID.atSetpoint());
    SmartDashboard.putNumber("Arm Tolerance", armPID.getPositionError());
    }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("\n\n\n\n Arm Command Has Finish \n\n\n\n\n");
    intakeSub.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
