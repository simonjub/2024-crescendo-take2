// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Climb;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  private final Climber m_climber = new Climber();

  private final CommandXboxController m_coDriverController = new CommandXboxController(1);

  private final DoubleSupplier climbAxis = () -> m_coDriverController.getRawAxis(5);

  public RobotContainer() {
    m_climber.setDefaultCommand(new Climb(m_climber, climbAxis));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
