// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  
private Swerve swerve;
public static boolean centric = true;

public static SendableChooser<Command> autoChooser;

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private final Joystick driver = new Joystick(OperatorConstants.kDriverControllerPort);
  // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);




  public RobotContainer() {
    swerve = new Swerve();
    swerve.setDefaultCommand(
      new SwerveCommand(
        swerve, 
        () -> driverController.getLeftY(), 
        () -> driverController.getLeftX(), 
        () -> driverController.getRightX(), 
        () -> centric));

    autoChooser = AutoBuilder.buildAutoChooser();

    Shuffleboard.getTab("Auton").add(autoChooser);

    configureBindings();
  }


  private void configureBindings() {
    driverController.leftBumper().onTrue(new InstantCommand(() -> centric = !centric));

    driverController.y().onTrue(swerve.resetGyro());
  }

  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
	}

  }

