// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
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

  /**
   * Creates a Swerve subsystem with the added kinematics.
   * 
   * @param kinematics
   */
  public SwerveSubsystem(SwerveDriveKinematics kinematics) {
    gyro = new AHRS(SPI.Port.kMXP);
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
        new Pose2d());

    calibrateGyro();
  }

  /**
   * Configures auto builder for pathplanner
   */
  public void configureAuto() {
    AutoBuilder.configureHolonomic(this::getCurrentPose, this::resetPose,
        this::getSpeeds, this::setStates,
        Constants.DriveConstants.kPathFollowerConfig,
        this);
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

  public void setStates(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    SmartDashboard.putNumber("Front Left Angle Setpoint", states[1].angle.getRadians());
    SmartDashboard.putNumber("Front Right Angle Setpoint", states[0].angle.getRadians());
    SmartDashboard.putNumber("Back Left Angle Setpoint", states[2].angle.getRadians());
    SmartDashboard.putNumber("Back Right Angle Setpoint", states[3].angle.getRadians());

    dashboard.putObject("Desired Front Left Power", states[1].speedMetersPerSecond);
    dashboard.putObject("Desired Front Right Power", states[0].speedMetersPerSecond);
    dashboard.putObject("Desired Back Left Power", states[2].speedMetersPerSecond);
    dashboard.putObject("Desired Back Right Power", states[3].speedMetersPerSecond);

    frontRightModule.setDesiredState(states[1]);
    frontLeftModule.setDesiredState(states[0]);
    backRightModule.setDesiredState(states[3]);
    backLeftModule.setDesiredState(states[2]);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Front Right Angle", (frontRightModule.getSteerPosition()));
    SmartDashboard.putNumber("Back Left Angle", (backLeftModule.getSteerPosition()));
    SmartDashboard.putNumber("Back Right Angle", (backRightModule.getSteerPosition()));
    SmartDashboard.putNumber("Front Left Angle", (frontLeftModule.getSteerPosition()));

    SmartDashboard.putNumber("Front Left Angle Raw", (frontLeftModule.getRawAbsoluteAngularPosition()));
    SmartDashboard.putNumber("Front Right Angle Raw", (frontRightModule.getRawAbsoluteAngularPosition()));
    SmartDashboard.putNumber("Back Left Angle Raw", (backLeftModule.getRawAbsoluteAngularPosition()));
    SmartDashboard.putNumber("Back Right Angle Raw", (backRightModule.getRawAbsoluteAngularPosition()));

    field2d.setRobotPose(getCurrentPose());
  }
}
