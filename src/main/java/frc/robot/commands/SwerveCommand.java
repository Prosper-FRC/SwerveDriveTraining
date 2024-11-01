// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveConstants;

public class SwerveCommand extends Command {
  /** Creates a new SwerveCommand. */
  private Swerve swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier centricSup;

  public SwerveCommand(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier centricSup) {
    this.swerve = swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.centricSup = centricSup;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translation = translationSup.getAsDouble();
    double strafe = strafeSup.getAsDouble();
    double rotation = rotationSup.getAsDouble();
    boolean centric = centricSup.getAsBoolean();

    swerve.drive(
      // The value as of now is between 0 and 1 // 
      // Multiplying by the max speed helps finding the real demand that was applied //
      new Translation2d(translation, strafe).times(SwerveConstants.maxSpeed), 
      rotation * SwerveConstants.maxAngularVelocity * 2, 
      centric
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
