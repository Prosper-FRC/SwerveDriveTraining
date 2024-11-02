// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.lib.DriveConversions;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  
  public int moduleNumber;
  public int driveMotorID;
  public int azimuthMotorID;
  public int CANCoderID;
  private Rotation2d angleOffset;
  public String name;

  public TalonFX driveMotor;
  public TalonFX azimuthMotor;
  public CANcoder CANCoder;

  PositionDutyCycle azimuthPID;
  VelocityVoltage drivePID;

  private CTREConfigs configs;

  public SwerveModule(int moduleNumber, int driveMotorID, int azimuthMotorID, int CANCoderID, Rotation2d angleOffset, String name) {
    this.moduleNumber = moduleNumber;
    this.driveMotorID = driveMotorID;
    this.azimuthMotorID = azimuthMotorID;
    this.CANCoderID = CANCoderID;
    this.angleOffset = angleOffset;
    this.name = name;

    azimuthPID = new PositionDutyCycle(0, 1, false, 0, 0, false, false, false);

    drivePID = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

    configs = new CTREConfigs(CANCoderID);

    driveMotor = new TalonFX(driveMotorID, "drivebase");
    configDrive();

    azimuthMotor = new TalonFX(azimuthMotorID, "drivebase");
    configAzimuth();

    CANCoder = new CANcoder(CANCoderID, "drivebase");
    configCANCoder();


    resetToAbsolute();

  }

  ////////////////////////////////////////// Hardware configurations: ////////////////////////////////////////////////////

  private void configCANCoder() {
    CANCoder.getConfigurator().apply(new CANcoderConfiguration());

    CANCoder.getConfigurator().apply(configs.swerveCanCoderConfig);
  }

  private void configDrive() {
    // driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    // driveMotor.clearStickyFault_BootDuringEnable(2.5);
    driveMotor.getConfigurator().apply(configs.swerveDriveFXConfig);
    driveMotor.setInverted(SwerveConstants.driveMotorInvert);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.setPosition(0.0);
  }

  private void configAzimuth() {
    // azimuthMotor.clearStickyFault_BootDuringEnable(2.5);
    azimuthMotor.getConfigurator().apply(configs.swerveAngleFXConfig);
    azimuthMotor.setInverted(SwerveConstants.angleMotorInvert);
    azimuthMotor.setNeutralMode(NeutralModeValue.Brake);
    azimuthMotor.setPosition(0.0);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(DriveConversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), SwerveConstants.wheelCircumference),  getAngle());
  }

  public double getRelativePos() {
    return azimuthMotor.getPosition().getValueAsDouble() * 360;
  }

  public double getAbsolutePos() {
    return CANCoder.getPosition().getValueAsDouble() * 360;
  }

  public TalonFX getDriveMotor() {
    return driveMotor;
  }

  public CANcoder getCANcoder() {
    return CANCoder;
  }


  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble());
  }



  public void setSpeed(SwerveModuleState desiredState) {
    double vel = DriveConversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
    driveMotor.setControl(drivePID.withVelocity(vel).withFeedForward(SwerveConstants.driveFF));
  }

  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getState().angle);

    azimuthMotor.setControl(azimuthPID.withPosition(state.angle.getRotations()));

    setSpeed(state);
  }

  public SwerveModuleState getState() {
    double velocity = DriveConversions.toMPS(driveMotor.getVelocity().getValueAsDouble(), SwerveConstants.wheelCircumference);
    Rotation2d theta = Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble());

    return new SwerveModuleState(velocity, theta);
  }

  public void resetToAbsolute() {
    Rotation2d actualAngle = (Rotation2d.fromRotations(CANCoder.getAbsolutePosition().getValueAsDouble())).minus(angleOffset);

    azimuthMotor.setPosition(actualAngle.getRotations());
  }
  

  @Override
  public void periodic() {

  }
}
