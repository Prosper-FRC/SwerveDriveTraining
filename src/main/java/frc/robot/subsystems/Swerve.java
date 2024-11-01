// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  private Pigeon2 gyro;
  SwerveModuleState[] states;
  SwerveModulePosition[] positions;
  SwerveModule[] mods;

  private SwerveDriveOdometry odometry;


  public Swerve() {

    /* Gyro configurations */
    gyro = new Pigeon2(SwerveConstants.pigeonID);
    gyro.getConfigurator();
    gyro.setYaw(0.0);

    odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), getPositions());

    mods = new SwerveModule[] {
      new SwerveModule(0, 11, 21, 31, Rotation2d.fromRotations(-0.333252), "Front Left"),
      new SwerveModule(1, 12, 22, 32, Rotation2d.fromRotations(0.316406), "Front Right"),
      new SwerveModule(2, 13, 23, 33, Rotation2d.fromRotations(-0.164307), "Back Left"),
      new SwerveModule(3, 14, 24, 34, Rotation2d.fromRotations(-0.390137), "Back Right"),
  
    };

    positions = new SwerveModulePosition[4];

    states = new SwerveModuleState[4];

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose,
      this::getSpeeds, 
      this::setSpeeds, 
      SwerveConstants.config, 
      this::shouldFlip,
      this);
  }

  ////////////////////////////////////////// All Pathplanner related methods: ////////////////////////////////////////////////////

  private Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  private void resetPose(Pose2d pose) {
    odometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    for (int i = 0; i < 4; i++) {
      states[i] = mods[i].getState();
    }

    ChassisSpeeds speeds = SwerveConstants.swerveKinematics.toChassisSpeeds(
      states[0], states[1], states[2], states[3]
    );
    return speeds;
  }

  // set desired state here
  public void setSpeeds(ChassisSpeeds desiredSpeeds) {

    for (int i = 0; i < 4; i++) {
      mods[i].setDesiredState(SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredSpeeds)[i]);
    }
  }

  private boolean shouldFlip() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    }
  

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  public Rotation2d getYaw() {
    if (SwerveConstants.invertGyro) {
      return Rotation2d.fromDegrees(360 - (gyro.getYaw().getValueAsDouble()));
    }

    else {
      return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }
  }

  public SwerveModulePosition[] getPositions() {
    for (SwerveModule module : mods) {
      positions[module.moduleNumber] = module.getPosition();
    }

    return positions;
    }

  ////////////////////////////////////////// Drive method: ////////////////////////////////////////////////////

   public void drive(Translation2d translation, double rotation, boolean centric) {
    ChassisSpeeds desiredChassisSpeeds;

    if (centric) {
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          rotation,
          getYaw());
      } 
      else {
        desiredChassisSpeeds = new ChassisSpeeds(
          translation.getX(), 
          translation.getY(),
          rotation
        );
      }

      SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
      
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

      for (int i = 0; i < 4; i++) {
        mods[1].setDesiredState(swerveModuleStates[i]);
      }
   
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public Command resetGyro() {
      return new InstantCommand(() -> gyro.setYaw(0.0));
    }


  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}