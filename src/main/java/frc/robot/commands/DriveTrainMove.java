// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSub;

public class DriveTrainMove extends CommandBase {
  /** Creates a new DriveTrainMove. */
  DriveTrainSub drivetrain;

  public DriveTrainMove(DriveTrainSub p_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
      drivetrain = p_drivetrain;
      addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.DriverStick.getY()/2;
    double turn = RobotContainer.DriverStick.getX();
    SmartDashboard.putNumber("Speed", speed);
    RobotContainer.drivetrain.arcadeDrive(speed, turn);
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
