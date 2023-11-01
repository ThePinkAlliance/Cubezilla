package frc.robot.subsystems;

import frc.robot.lib.IO;

public abstract class IntakeIO extends IO<IntakeIOInputs> {
    public abstract void setVelocity(double speed);
}
