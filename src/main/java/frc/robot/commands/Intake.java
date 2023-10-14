// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.IntakeSubsystem;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake extends CommandBase {
  private IntakeSubsystem subsystem;
  private TalonFX motor;
  private double speed;
  /** Creates a new Intake. */
  public Intake(IntakeSubsystem subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.speed = speed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
