package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
  public double getDrivePosition();

  public double getSteerPosition();

  public double getDriveVelocity();

  public double getSteerVelocity();

  public double getSteerError();

  /**
   * Reset the drive & steer encoders.
   */
  public void resetEncoders();

  /**
   * Current pod angle in rad/s.
   */
  public double getAbsoluteEncoderAngle();

  /**
   * Returns the current state of the swerve pod.
   */
  public SwerveModuleState getState();

  /**
   * Returns the current position (drive pos, steer angle) of the pod.
   */
  public SwerveModulePosition getPosition();

  public double getRawAbsoluteAngularPosition();

  /**
   * Stop the pod.
   */
  public void stop();

  /**
   * Sets the current swerve pod state to the prescribed one.
   * 
   * @param state Desired swerve pod.
   */
  public void setDesiredState(SwerveModuleState state);
}
