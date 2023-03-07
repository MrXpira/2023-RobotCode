// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.apache.commons.collections4.sequence.InsertCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX motorBottom; 
  TalonFX motorTop;
  DoubleSolenoid arm;
  /** Creates a new Shooter. */
  public Shooter() {
    motorTop = new TalonFX(Constants.ShooterConstants.motorTopID);
    motorBottom = new TalonFX(Constants.ShooterConstants.motorBottomID);
    arm = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    arm.set(Value.kForward);

    motorTop.setNeutralMode(NeutralMode.Coast);
    motorBottom.setNeutralMode(NeutralMode.Coast);
    
    motorTop.setInverted(TalonFXInvertType.Clockwise);
    motorBottom.setInverted(TalonFXInvertType.CounterClockwise);
  }

  public Command lowerShooter() {
    return this.run(() -> arm.set(Value.kReverse));
  }

  public Command raiseShooter() {
    return this.run(() -> arm.set(Value.kForward));
  }

  public Command runIntake() {
    return this.run(() -> motorTop.set(ControlMode.PercentOutput, .5));
  }

  public Command intakeCube() {
    return this.run(() -> lowerShooter()).alongWith(runIntake());
  }

  public Command shootLower() {
    return this.run(() -> raiseShooter()).andThen(() -> {
      motorTop.set(ControlMode.Velocity, Constants.ShooterConstants.bottomGoalVelocityTopMotor);
      motorBottom.set(ControlMode.Velocity, Constants.ShooterConstants.bottomGoalVelocityBottomMotor);
    });
  }

  public Command shootHigh() {
    return this.run(() -> raiseShooter()).andThen(() -> {
      motorTop.set(ControlMode.Velocity, Constants.ShooterConstants.highGoalVelocityBottomMotor);
      motorBottom.set(ControlMode.Velocity, Constants.ShooterConstants.highGoalVelocityBottomMotor);
    });
  }

  public Command defaultState() {
    return this.run(() -> raiseShooter()).alongWith(new InstantCommand(() -> {
        motorBottom.set(ControlMode.PercentOutput, 0);
        motorTop.set(ControlMode.PercentOutput, 0);
      })
    );
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("CurrentSpeed Top", motorTop.getSelectedSensorVelocity());
    SmartDashboard.putNumber("CurrentSpeed Bottom", motorBottom.getSelectedSensorVelocity());
  }
}
