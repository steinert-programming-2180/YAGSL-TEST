// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(Constants.DrivebaseConstants.TRANSLATION_SCALE)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
      m_driverController::getRightX,
      m_driverController::getRightY)
      .headingWhile(true);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.DEADBAND),
      () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.DEADBAND),
      () -> m_driverController.getRightX(),
      () -> m_driverController.getRightY());

  public RobotContainer() {
    configureBindings();

    if (RobotBase.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Example Auto", Autos.exampleAuto(m_exampleSubsystem));
    autoChooser.addOption("Drive Forward (1s)", drivebase.driveForward().withTimeout(1));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_driverController.rightBumper().whileTrue(intakeSubsystem.bottomClockwiseCommand());
    m_driverController.leftBumper().whileTrue(intakeSubsystem.bottomCounterClockwiseCommand());
    m_driverController.rightTrigger().whileTrue(intakeSubsystem.triggerIntakeCommand());
    m_driverController.leftTrigger().whileTrue(intakeSubsystem.topClockwiseCommand());
    m_driverController.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

    if (RobotBase.isSimulation()) {
      m_driverController.start().onTrue(Commands.runOnce(
          () -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
