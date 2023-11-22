// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.modules;

import com.ThePinkAlliance.core.util.Gains;
import com.ThePinkAlliance.core.util.GainsFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveModule;

/** Add your docs here. */
public class FX_SwerveModule implements SwerveModule {
  private PIDController steerController;

  private WPI_TalonFX driveMotor;
  private WPI_TalonFX steerMotor;
  private WPI_CANCoder canCoder;

  private double absoluteEncoderOffsetRad;

  private final int SLOT_IDX;
  private final double MAX_VELOCITY_DRIVE_TICKS;

  public FX_SwerveModule(int steerId, int driveId, int canCoderId, boolean invertDrive, boolean invertSteer,
      double absoluteEncoderOffsetRad,
      Gains steerGains, GainsFX driveGains, String network) {
    this.canCoder = new WPI_CANCoder(canCoderId, network);
    this.steerMotor = new WPI_TalonFX(steerId, network);
    this.driveMotor = new WPI_TalonFX(driveId, network);

    this.SLOT_IDX = 0;

    this.driveMotor.config_kP(SLOT_IDX, driveGains.kP);
    this.driveMotor.config_kI(SLOT_IDX, driveGains.kI);
    this.driveMotor.config_kD(SLOT_IDX, driveGains.kD);
    this.driveMotor.config_kF(SLOT_IDX, driveGains.kF);
    this.driveMotor.config_IntegralZone(SLOT_IDX, driveGains.kIzone);
    this.driveMotor.configPeakOutputForward(driveGains.kPeakOutput);
    this.driveMotor.configPeakOutputForward(-driveGains.kPeakOutput);

    this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

    this.steerController = new PIDController(steerGains.kP, steerGains.kI, steerGains.kD);
    this.steerController.enableContinuousInput(-Math.PI, Math.PI);

    this.driveMotor.setInverted(invertDrive);
    this.steerMotor.setInverted(invertSteer);

    // I Think this calculation works for max drive velocity in ticks.
    this.MAX_VELOCITY_DRIVE_TICKS = ((Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        / (Constants.ModuleConstants.kWheelDiameterMeters * Math.PI) / Constants.ModuleConstants.kDriveMotorGearRatio)
        * 2048);

    resetEncoders();
  }

  /**
   * Returns the drive wheel position in meters.
   */
  @Override
  public double getDrivePosition() {
    return ((driveMotor.getSelectedSensorPosition() / 2048.0) * Constants.ModuleConstants.kDriveMotorGearRatio)
        * (Constants.ModuleConstants.kWheelDiameterMeters * Math.PI);
  }

  /*
   * This returns the current position of the steer shaft in radians.
   */
  @Override
  public double getSteerPosition() {
    double rad = Math.toRadians(canCoder.getAbsolutePosition()) - absoluteEncoderOffsetRad;

    return rad;
  }

  @Override
  public double getRawAbsoluteAngularPosition() {
    return Math.toRadians(canCoder.getAbsolutePosition());
  }

  @Override
  public double getDriveVelocity() {
    /*
     * It might be necessary to change the constant because it does not take into
     * account the gear ratio.
     */
    return driveMotor.getSelectedSensorVelocity() * Constants.ModuleConstants.kDriveMotorGearRatio;
  }

  /**
   * Returns the velocity of the steer motor in rad/sec.
   */
  @Override
  public double getSteerVelocity() {
    return Math.toRadians(canCoder.getVelocity());
  }

  @Override
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    steerMotor.setSelectedSensorPosition(getAbsoluteEncoderAngle());
  }

  @Override
  public double getAbsoluteEncoderAngle() {
    double angle = canCoder.getAbsolutePosition();

    angle *= (2.0 * Math.PI / 180);
    angle -= absoluteEncoderOffsetRad;

    return angle;
  }

  /*
   * Returns the current state of the swerve module using velocity.
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
  }

  /*
   * Returns the current state of the swerve module using position.
   */
  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerPosition()));
  }

  /**
   * Sets the current module state to the desired one.
   * 
   * @param state desired swerve module state
   */
  @Override
  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getState().angle);

    /*
     * I think dividing the desired velocity will work. NOTE: I would like to
     * research and find out.
     */
    double desiredVelocity = (state.speedMetersPerSecond / Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
        * MAX_VELOCITY_DRIVE_TICKS / 100;

    driveMotor.set(ControlMode.Velocity,
        desiredVelocity);

    double output = steerController.calculate(getSteerPosition(), state.angle.getRadians());
    steerMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    steerMotor.set(ControlMode.PercentOutput, 0);
  }
}
