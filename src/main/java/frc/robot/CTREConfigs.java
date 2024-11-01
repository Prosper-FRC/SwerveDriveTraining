package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.subsystems.SwerveConstants;

// Taken from robot-main-redux:
public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(int CANcoderID){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        //Angle
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();

        angleSupplyLimit.StatorCurrentLimit = SwerveConstants.angleStatorCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = SwerveConstants.angleSupplyCurrentLimit;
        angleSupplyLimit.StatorCurrentLimitEnable = SwerveConstants.angleEnableStatorLimit;
        angleSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.angleEnableSupplyLimit;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        swerveAngleFXConfig.Slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.angleKD;
        swerveAngleFXConfig.Slot0.kS = SwerveConstants.angleKF;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // swerveAngleFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // swerveAngleFXConfig.Feedback.FeedbackRemoteSensorID = CANcoderID;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.angleGearRatio;
    
        //Drive
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.StatorCurrentLimit = SwerveConstants.driveStatorCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = SwerveConstants.driveSupplyCurrentLimit;
        driveSupplyLimit.StatorCurrentLimitEnable = SwerveConstants.driveEnableStatorLimit;
        driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.driveEnableSupplyLimit;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;

        swerveDriveFXConfig.Slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.driveKD;
        swerveDriveFXConfig.Slot0.kS = SwerveConstants.driveKF;

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.driveGearRatio;

        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;

        //Cancoder
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }
}