package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmPivot extends CommandBase {
    private ArmSubsystem subsystem;

    private double armPos;

    public ArmPivot(ArmSubsystem subsystem, double armPos) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
        this.armPos = armPos;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        subsystem.setDesiredPos(armPos);
    }

}
