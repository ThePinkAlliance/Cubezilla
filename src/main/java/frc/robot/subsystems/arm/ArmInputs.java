package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmInputs implements LoggableInputs {
    public double armSetpoint;
    public double armPosition;

    @Override
    public void toLog(LogTable table) {
        table.put("arm_setpoint", armSetpoint);
        table.put("arm_position", armPosition);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("arm_setpoint");
        table.get("arm_position");
    }
}
