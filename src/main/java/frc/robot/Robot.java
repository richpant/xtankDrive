// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_leftMotor2 = new PWMSparkMax(1);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(2);
  private final PWMSparkMax m_rightMotor2 = new PWMSparkMax(3);
  private final MotorControllerGroup m_LeftMotor = new MotorControllerGroup(m_leftMotor, m_leftMotor2);
  private final MotorControllerGroup m_RightMotor = new MotorControllerGroup(m_rightMotor, m_rightMotor2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_LeftMotor, m_RightMotor);
  private final XboxController m_driverController = new XboxController(0);
  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    m_rightMotor2.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with tank drive.
    // That means that the Y axis of the left stick moves the left side
    // of the robot forward and backward, and the Y axis of the right stick
    // moves the right side of the robot forward and backward.
    m_robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY());
    m_solenoid.set(m_driverController.getAButton());
  }
}
