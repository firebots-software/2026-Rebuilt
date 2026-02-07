// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.TestArm;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final TestArm testArm = new TestArm();

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    joystick.a().onTrue(testArm.SetAngle());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
