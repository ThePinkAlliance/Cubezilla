package frc.robot.subsystems.drive.arm;

import java.security.PublicKey;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{

    private CANSparkMax leftSparxMax;
    private CANSparkMax rightSparxMax;

    private int speedRange = 40;

    public void setDesiredPos(double desiredPos){
        final double leftNeoMax = -27.095;
        final double rightNeoMax = 27.0712;
        final double leftNeoMin = -15;
        final double rightNeoMin = 15;

        leftSparxMax.getPIDController().setReference(desiredPos, CANSparkMax.ControlType.kPosition);
    }



    public ArmSubsystem() {
        this.leftSparxMax = new CANSparkMax(Constants.DriveConstants.LeftArmRotateMotor, MotorType.kBrushless);
        this.rightSparxMax = new CANSparkMax(Constants.DriveConstants.RightArmRotateMotor, MotorType.kBrushless);

        leftSparxMax.getEncoder().setPosition(0);
        rightSparxMax.getEncoder().setPosition(0);

        rightSparxMax.follow(leftSparxMax, true);

        leftSparxMax.getPIDController().setP(1);

        leftSparxMax.getPIDController().setOutputRange(-speedRange, speedRange);
    }

    public void periodic(){
        double leftArmEncoderValue = leftSparxMax.getEncoder().getPosition();
        double rightArmEncoderValue = rightSparxMax.getEncoder().getPosition();

        
        SmartDashboard.putNumber("left Arm Neo Encoder Value", (leftArmEncoderValue));
        SmartDashboard.putNumber("Right Arm Neo Encoder Value", (rightArmEncoderValue));
    }
}