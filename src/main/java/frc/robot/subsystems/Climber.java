// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Follows climber_right */
  private CANSparkMax m_climber_left =
      new CANSparkMax(Constants.ClimberConstants.kLeftClimberId, MotorType.kBrushless);

  private CANSparkMax m_climber_right =
      new CANSparkMax(Constants.ClimberConstants.kRightClimberId, MotorType.kBrushless);

  private double rampRate = 1.5;

  /** Creates a new Climber. */
  public Climber() {
    m_climber_left.restoreFactoryDefaults();
    m_climber_right.restoreFactoryDefaults();

    m_climber_left.follow(m_climber_right, true);

    m_climber_left.setOpenLoopRampRate(rampRate);
    m_climber_right.setOpenLoopRampRate(rampRate);

    m_climber_left.setIdleMode(IdleMode.kBrake);
    m_climber_right.setIdleMode(IdleMode.kBrake);

    m_climber_left.burnFlash();
    m_climber_right.burnFlash();
  }

  @Override
  public void periodic() {}
}
