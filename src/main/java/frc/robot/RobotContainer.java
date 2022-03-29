// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.commands.DriveTrainMove;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // Motors
  public static MotorController leftFrontMotor;
  public static MotorController leftMiddleMotor;
  public static MotorController leftBackMotor;
  public static MotorController rightFrontMotor;
  public static MotorController rightMiddleMotor;
  public static MotorController rightBackMotor;

  // Joystick
  public static Joystick DriverStick;

  // Motor Groups & Differential Drive
  public static MotorControllerGroup leftMotors;
  public static MotorControllerGroup rightMotors;

  public static DifferentialDrive differentialDrive;
  public static DriveTrainSub drivetrain = new DriveTrainSub();
  private final DriveTrainMove driveTrainMove = new DriveTrainMove(drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // LEFT MOTORS
    leftFrontMotor = new WPI_TalonFX(Constants.LEFT_FRONT_MOTOR);
    leftMiddleMotor = new WPI_TalonFX(Constants.LEFT_MIDDLE_MOTOR);
    leftBackMotor = new WPI_TalonFX(Constants.LEFT_BACK_MOTOR);
    leftMotors = new MotorControllerGroup(leftFrontMotor, leftMiddleMotor, leftBackMotor);

    // RIGHT MOTORS
    rightFrontMotor = new WPI_TalonFX(Constants.RIGHT_FRONT_MOTOR);
    rightMiddleMotor = new WPI_TalonFX(Constants.RIGHT_MIDDLE_MOTOR);
    rightBackMotor = new WPI_TalonFX(Constants.RIGHT_BACK_MOTOR);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightMiddleMotor, rightBackMotor);
    rightMotors.setInverted(true);

    // DIFFERENTIAL DRIVE
    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    // JOYSTICK
    DriverStick = new Joystick(Constants.JOYSTICK);
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveTrainMove);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
