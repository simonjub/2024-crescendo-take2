// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Climb;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.swerve.CTREConfigs;
import frc.robot.subsystems.swerve.Swerve;
<<<<<<< HEAD
import java.util.function.DoubleSupplier;
=======
import frc.robot.commands.SpeakerLock;
import org.photonvision.PhotonCamera;
>>>>>>> afe347c (WIP acquiring apriltag target)

public class RobotContainer {
  private final Climber m_climber = new Climber();

  private final Swerve m_swerveDrive = new Swerve();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

  private final double kJoystickDeadband = 0.1;

  private double conditionJoystick(int axis, SlewRateLimiter limiter, double deadband) {
    return -limiter.calculate(
        MathUtil.applyDeadband(m_driverController.getRawAxis(axis), deadband));
  }

<<<<<<< HEAD
  private final CommandXboxController m_coDriverController = new CommandXboxController(1);

  private final DoubleSupplier climbAxis = () -> m_coDriverController.getRawAxis(5);
=======
  private SpeakerLock m_speakerLockCmd;
  private PhotonCamera m_camera;
>>>>>>> afe347c (WIP acquiring apriltag target)

  public RobotContainer() {
    m_climber.setDefaultCommand(new Climb(m_climber, climbAxis));

    m_swerveDrive.resetModulesToAbsolute();

    m_swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            m_swerveDrive,
            () -> conditionJoystick(translationAxis, translationLimiter, kJoystickDeadband),
            () -> conditionJoystick(strafeAxis, strafeLimiter, kJoystickDeadband),
            () -> conditionJoystick(rotationAxis, rotationLimiter, kJoystickDeadband),
            () -> true));

    m_camera = new PhotonCamera("front");
    m_speakerLockCmd = new SpeakerLock(null, null, m_camera);
    configureBindings();
    configureSmartDashboardCommands();
  }

  private void configureBindings() {}

  private void configureSmartDashboardCommands() {
    SmartDashboard.putString("Camera", m_camera.toString());
    SmartDashboard.putData(m_speakerLockCmd);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
