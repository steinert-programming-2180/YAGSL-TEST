package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final File directory =
      new File(Filesystem.getDeployDirectory(), Constants.DrivebaseConstants.SWERVE_DIRECTORY);
  private final SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, Pose2d.kZero);
    } catch (Exception e) {
      throw new RuntimeException("Failed to initialize swerve drive from deploy/swerve", e);
    }

    swerveDrive.setModuleStateOptimization(true);
    setupPathPlanner();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public void driveFieldOrientedSetpoint(ChassisSpeeds speeds) {
    swerveDrive.driveFieldOriented(speeds);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public Command driveFieldOriented(SwerveInputStream inputStream) {
    return driveFieldOriented((Supplier<ChassisSpeeds>) inputStream);
  }

  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    SwerveInputStream inputStream =
        SwerveInputStream.of(swerveDrive, translationX, translationY)
            .withControllerHeadingAxis(headingX, headingY)
            .deadband(Constants.OperatorConstants.DEADBAND)
            .scaleTranslation(Constants.DrivebaseConstants.TRANSLATION_SCALE)
            .allianceRelativeControl(true)
            .headingWhile(true);
    return driveFieldOriented(inputStream);
  }

  public Command driveForward() {
    return run(
        () ->
            swerveDrive.driveFieldOriented(
                new ChassisSpeeds(1.0, 0.0, 0.0)));
  }

  public Command lock() {
    return run(swerveDrive::lockPose);
  }

  public Command zeroGyroWithAllianceCommand() {
    return runOnce(
        () -> {
          zeroGyro();
          DriverStation.getAlliance()
              .filter(alliance -> alliance == DriverStation.Alliance.Red)
              .ifPresent(
                  alliance ->
                      resetOdometry(
                          new Pose2d(getPose().getTranslation(), Rotation2d.k180deg)));
        });
  }

  public Command resetOdometryCommand(Pose2d pose) {
    return runOnce(() -> resetOdometry(pose));
  }

  public void setupPathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          swerveDrive::getPose,
          swerveDrive::resetOdometry,
          swerveDrive::getRobotVelocity,
          swerveDrive::setChassisSpeeds,
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () ->
              DriverStation.getAlliance()
                  .map(alliance -> alliance == DriverStation.Alliance.Red)
                  .orElse(false),
          this);
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
    }
  }

  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints =
        new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(),
            4.0,
            swerveDrive.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose, constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0));
  }

  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    var setpointGenerator =
        new com.pathplanner.lib.util.swerve.SwerveSetpointGenerator(
            RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<com.pathplanner.lib.util.swerve.SwerveSetpoint> prevSetpoint =
        new AtomicReference<>(
            new com.pathplanner.lib.util.swerve.SwerveSetpoint(
                swerveDrive.getRobotVelocity(),
                swerveDrive.getStates(),
                com.pathplanner.lib.util.DriveFeedforwards.zeros(
                    swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(
        () -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          var newSetpoint =
              setpointGenerator.generateSetpoint(
                  prevSetpoint.get(),
                  robotRelativeChassisSpeed.get(),
                  newTime - previousTime.get());
          swerveDrive.drive(
              newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);
        });
  }

  public Command driveWithSetpointGeneratorFieldRelative(
      Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(
          () -> ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading()));
    } catch (IOException | ParseException e) {
      DriverStation.reportError(e.toString(), true);
      return Commands.none();
    }
  }
}
