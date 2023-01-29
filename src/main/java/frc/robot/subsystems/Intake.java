// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase implements AutoCloseable{
  private final CANSparkMax m_motor;
  private final DoubleSolenoid m_piston;

  

  /** Creates a new Intake. */
  public Intake() {
    m_motor = new CANSparkMax(Constants.IntakeConstants.motorID, MotorType.kBrushless);
    m_piston = new DoubleSolenoid(
                                PneumaticsModuleType.REVPH, 
                                Constants.IntakeConstants.kPistonFwdChannel, 
                                Constants.IntakeConstants.kPistonRevChannel);    
    m_piston.set(Value.kForward);
  }

  public void deploy() {
    m_piston.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    m_piston.set(DoubleSolenoid.Value.kReverse);
    m_motor.set(0); // turn off the motor
  }

  public void activate(double speed) {
    if (isDeployed()) {
      m_motor.set(speed);
    } else { // if piston isn't open, do nothing
      m_motor.set(0);
    }
  }

  public boolean isDeployed() {
    return m_piston.get() == DoubleSolenoid.Value.kForward;
  }

  @Override
  public void close() throws Exception {
    m_piston.close();
    m_motor.close();
  }

  public double getSpeedPercentage() {
    return m_motor.get();
  }
  
  public CommandBase retractIntake() {
    return this.runOnce(() -> {
      this.retract();
    });
  }
  public CommandBase deployIntake(double speed) {
    return this.runOnce(() -> {
      this.deploy();
    });
  }
}
