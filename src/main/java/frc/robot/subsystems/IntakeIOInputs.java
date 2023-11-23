package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * All inputs for the intake subsytem.
 */
@AutoLog
public class IntakeIOInputs implements LoggableInputs {
    public double intakeVelocity;

    @Override
    public void toLog(LogTable table) {
        table.put("intakeVelocity", intakeVelocity);
    }

    @Override
    public void fromLog(LogTable table) {
        intakeVelocity = table.getDouble("intakeVelocity", intakeVelocity);
    }
}