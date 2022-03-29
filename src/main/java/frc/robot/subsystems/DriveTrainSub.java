// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveTrainSub extends SubsystemBase {
  /** Creates a new DriveTrainSub. */
  public DriveTrainSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double t_speed, double t_turn){
    double speed = t_speed;
    double turn = 0;
    SmartDashboard.putNumber("Speed", speed);
    RobotContainer.differentialDrive.arcadeDrive(speed, turn);

}

}
