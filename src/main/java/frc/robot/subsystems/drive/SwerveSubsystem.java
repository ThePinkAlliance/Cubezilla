// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.lib.Dashboard;
import frc.robot.subsystems.drive.modules.REV_SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;

  private AHRS gyro;
  private Field2d field2d;
  private Dashboard dashboard;
  // private DataLog log;
  // private DoubleLogEntry xLogEntry;
  // private DoubleLogEntry yLogEntry;
  private SwerveModule[] modules;
  private SwerveModulePosition[] lastModulePositionsMeters;
  private Rotation2d lastGyroYaw;

  /**
   * Creates a Swerve subsystem with the added kinematics.
   * 
   * @param kinematics
   */
  public SwerveSubsystem(SwerveDriveKinematics kinematics) {
    // DataLogManager.start();

    // log = DataLogManager.getLog();

    // xLogEntry = new DoubleLogEntry(log, "/dt/xPos");
    // yLogEntry = new DoubleLogEntry(log, "/dt/yPos");

    this.gyro = new AHRS(SPI.Port.kMXP);
    this.field2d = new Field2d();
    this.dashboard = new Dashboard("Swerve");

    this.frontRightModule = new REV_SwerveModule(DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightTurningReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains);

    this.frontLeftModule = new REV_SwerveModule(DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftTurningReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains);

    this.backRightModule = new REV_SwerveModule(DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveMotorPort, DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightTurningReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains);

    this.backLeftModule = new REV_SwerveModule(DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftTurningReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains);

    this.kinematics = kinematics;

    this.estimator = new SwerveDrivePoseEstimator(
        kinematics, getRotation(), new SwerveModulePosition[] { frontRightModule.getPosition(),
            frontLeftModule.getPosition(), backRightModule.getPosition(),
            backLeftModule
                .getPosition() },
        new Pose2d(), VecBuilder.fill(0.4, 0.4, 0.4),
        VecBuilder.fill(0.9, 0.9, 0.9));
    this.modules = new SwerveModule[] { frontRightModule, frontLeftModule, backRightModule, backLeftModule };
    this.lastModulePositionsMeters = getPositions();

    SmartDashboard.putData("Field", field2d);

    calibrateGyro();
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        frontRightModule.getPosition(),
        frontLeftModule.getPosition(),
        backRightModule.getPosition(),
        backLeftModule.getPosition()
    };
  }

  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
        frontRightModule.getState(),
        frontLeftModule.getState(),
        backRightModule.getState(),
        backLeftModule.getState()
    };
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public void calibrateGyro() {
  }

  public void setGyro(double angle) {
    this.gyro.setAngleAdjustment(angle);
  }

  public Field2d getField2d() {
    return field2d;
  }

  public void resetGyro() {
    this.gyro.reset();
  }

  private Twist2d scaleTwist2d(Twist2d twist2d, double factor) {
    return new Twist2d(twist2d.dx * factor, twist2d.dy * factor, twist2d.dtheta * factor);
  }

  public void setStates(ChassisSpeeds speeds) {
    // Looper is how far into the future are we looking
    double looper = .01;

    /*
     * Check the angular drift with this solution & if I doesn't work explore the
     * possiblity of steer error in swerve pods.
     */
    double gyro_update_rate = gyro.getRequestedUpdateRate();
    Pose2d currentPose = getCurrentPose();
    Pose2d desired = new Pose2d(currentPose.getX() + (speeds.vxMetersPerSecond *
        looper),
        currentPose.getY() + (speeds.vyMetersPerSecond * looper),
        currentPose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond)));

    Twist2d twist_vel = scaleTwist2d(currentPose.log(desired), 1);
    ChassisSpeeds updated_speeds = new ChassisSpeeds(twist_vel.dx / looper,
        twist_vel.dy / looper,
        twist_vel.dtheta);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    dashboard.putObject("Desired Front Left Power", states[1].speedMetersPerSecond);
    dashboard.putObject("Desired Front Right Power", states[0].speedMetersPerSecond);
    dashboard.putObject("Desired Back Left Power", states[2].speedMetersPerSecond);
    dashboard.putObject("Desired Back Right Power", states[3].speedMetersPerSecond);

    frontRightModule.setDesiredState(states[1]);
    frontLeftModule.setDesiredState(states[0]);
    backRightModule.setDesiredState(states[3]);
    backLeftModule.setDesiredState(states[2]);

    field2d.setRobotPose(getCurrentPose());
  }

  public void resetPose(Pose2d pose2d) {
    estimator.resetPosition(getRotation(), getPositions(), pose2d);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public Pose2d getCurrentPose() {
    return estimator.getEstimatedPosition();
  }

  private double lastEpoch = 0;
  private double lastAngularPos = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Front Right Velocity", (frontRightModule.getDriveVelocity()));
    SmartDashboard.putNumber("Back Left Velocity", (backLeftModule.getDriveVelocity()));
    SmartDashboard.putNumber("Back Right Velocity", (backRightModule.getDriveVelocity()));
    SmartDashboard.putNumber("Front Left Velocity", (frontLeftModule.getDriveVelocity()));

    if (lastEpoch != 0) {
      double currentAngularPos = gyro.getAngle();
      SmartDashboard.putNumber("Angular Vel Rad/s",
          ((currentAngularPos - lastAngularPos) * (Math.PI / 180)) / (Timer.getFPGATimestamp() - lastEpoch));
      lastAngularPos = currentAngularPos;
    }

    SmartDashboard.putNumber("Back Right Heading", backRightModule.getRawAbsoluteAngularPosition());
    SmartDashboard.putNumber("Heading", getHeading());

    Pose2d pose = getCurrentPose();

    // if (pose.getX() != 0 && pose.getY() != 0) {
    // xLogEntry.append(getCurrentPose().getX());
    // yLogEntry.append(getCurrentPose().getY());
    // }

    field2d.setRobotPose(getCurrentPose());
    estimator.update(getRotation(), getPositions());

    lastEpoch = Timer.getFPGATimestamp();
  }
}
