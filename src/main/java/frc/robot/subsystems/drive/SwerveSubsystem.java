// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.drive.modules.WPI_SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;

  private AHRS gyro;

  /** Creates a new DrivetrainSubsystem. */
  public SwerveSubsystem(SwerveDriveKinematics kinematics) {
    gyro = new AHRS(SPI.Port.kMXP);

    this.frontRightModule = new WPI_SwerveModule(DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightTurningReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains, "base");

    this.frontLeftModule = new WPI_SwerveModule(DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftTurningReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains, "base");

    this.backRightModule = new WPI_SwerveModule(DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveMotorPort, DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightTurningReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains, "base");

    this.backLeftModule = new WPI_SwerveModule(DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftTurningReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains, "base");

    this.kinematics = kinematics;

    this.estimator = new SwerveDrivePoseEstimator(
        kinematics, getRotation(), new SwerveModulePosition[] { frontRightModule.getPosition(),
            frontLeftModule.getPosition(), backRightModule.getPosition(),
            backLeftModule
                .getPosition() },
        new Pose2d());
  }

  public List<SwerveModulePosition> getPositions() {
    return List.of(frontRightModule.getPosition(), frontLeftModule.getPosition(), backRightModule.getPosition(),
        backLeftModule.getPosition());
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public void calibrateGyro() {
    this.gyro.calibrate();
  }

  public void setGyro(double angle) {
    this.gyro.setAngleAdjustment(angle);
  }

  public void resetGyro() {
    this.gyro.reset();
  }

  public void setStates(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    SmartDashboard.putNumber("Desired Front Left Angle", states[1].angle.getDegrees());
    SmartDashboard.putNumber("Desired Front Right Angle", states[0].angle.getDegrees());
    SmartDashboard.putNumber("Desired Back Left Angle", states[2].angle.getDegrees());
    SmartDashboard.putNumber("Desired Back Right Angle", states[3].angle.getDegrees());

    SmartDashboard.putNumber("Desired Front Left Power", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Desired Front Right Power", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Desired Back Left Power", states[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Desired Back Right Power", states[3].speedMetersPerSecond);

    frontRightModule.setDesiredState(states[0]);
    frontLeftModule.setDesiredState(states[1]);
    backRightModule.setDesiredState(states[2]);
    backLeftModule.setDesiredState(states[3]);
  }

  public Pose2d getCurrentPose() {
    return estimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Front Left Angle", frontLeftModule.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Front Right Angle", frontRightModule.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Back Left Angle", backLeftModule.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Back Right Angle", backRightModule.getAbsoluteEncoderAngle());
  }
}
