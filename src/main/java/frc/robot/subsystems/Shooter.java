// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  TalonFX motorBottom; 
  TalonFX motorTop;
  TalonFX rotateMotor;
  TalonFX rotateMotorFollower;
  ArmFeedforward m_armFF;

  /** Creates a new Shooter. */
  public Shooter() {
    motorTop = new TalonFX(Constants.ShooterConstants.motorTopID);
    motorBottom = new TalonFX(Constants.ShooterConstants.motorBottomID);
    rotateMotor = new TalonFX(Constants.ShooterConstants.rotateMotor);
    rotateMotorFollower = new TalonFX(Constants.ShooterConstants.rotateMotorFollower);
    
    m_armFF = new ArmFeedforward(Constants.ShooterConstants.armkS, Constants.ShooterConstants.armkG, 0);

    /* Config Intake Motors */
    motorTop.setNeutralMode(NeutralMode.Coast);
    motorBottom.setNeutralMode(NeutralMode.Coast);
    

    motorTop.setInverted(TalonFXInvertType.Clockwise);
    motorBottom.setInverted(TalonFXInvertType.Clockwise);

    /* Config Arm Motor */
    rotateMotor.configAllSettings(Robot.ctreConfigs.shooterArmFXConfig);
    rotateMotor.setInverted(TalonFXInvertType.Clockwise);
    rotateMotor.configMotionSCurveStrength(0);
    rotateMotor.setNeutralMode(NeutralMode.Brake);
    
    /* Follow Arm Motor */
    rotateMotorFollower.follow(rotateMotor);
    rotateMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);   
    rotateMotorFollower.setNeutralMode(NeutralMode.Brake);

		/* Zero the sensor once on robot boot up */
		rotateMotor.setSelectedSensorPosition(0, Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kTimeoutMs);
  }

  @Override
  public void periodic() {

      // ShuffleboardTab tab = Shuffleboard.getTab("Arm");
      SmartDashboard.putNumber("Shooter Master Falcon Position", rotateMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Shooter Follower Falcon Position", rotateMotorFollower.getSelectedSensorPosition());
      SmartDashboard.putNumber("Shooter Follower Falcon Voltage", rotateMotor.getStatorCurrent());
      
      //  SmartDashboard.putNumber("kP", kP.getDouble(.5));

      SmartDashboard.putNumber("Current Arm", rotateMotor.getStatorCurrent());


      // tab.add(this);
      // tab.add("Position Of Arm", rotateMotor.getSelectedSensorPosition()).withWidget(BuiltInWidgets.kGraph);
      // //Shuffleboard.getTab("Arm").add("P", 
      // tab.add("Motor Controller", rotateMotor.getMotorOutputPercent()).withWidget(BuiltInWidgets.kMotorController);
      // tab.add("FeedForward", calculateFeedForward()).withWidget(BuiltInWidgets.kGraph);

      // kP = tab.add("P", .05).getEntry();

      
  }

  public Command moveArm(DoubleSupplier percent) {
    return this.run(() -> {
      rotateMotor.set(ControlMode.PercentOutput, percent.getAsDouble());
      rotateMotorFollower.follow(rotateMotor);
    });
  }

  public void moveArmToPosition(double targetPos) {
    double armMotorHorizontalOffset = 9000;
      rotateMotor.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, 
                      m_armFF.calculate(((2*Math.PI) / 2048 / 16) * (rotateMotor.getSelectedSensorPosition() - armMotorHorizontalOffset), 
                      rotateMotor.getSelectedSensorVelocity()));

      rotateMotorFollower.follow(rotateMotor);

      System.out.println("Move to: " + targetPos + "Current: " + rotateMotor.getSelectedSensorPosition());  
  }

  public Command shoot(double speedTop, double speedBottom) {
    return this.runEnd(() -> {
      motorTop.set(ControlMode.PercentOutput, -speedTop);
      motorBottom.set(ControlMode.PercentOutput, -speedBottom);
    },(() -> {
      motorTop.set(ControlMode.PercentOutput, 0);
      motorBottom.set(ControlMode.PercentOutput, 0);
    }));
  }

  public Command intake() {
    return this.runEnd(() -> {
        moveArmToPosition(9000);

        motorTop.set(ControlMode.PercentOutput, Constants.ShooterConstants.intakeVelocity);
        motorBottom.set(ControlMode.PercentOutput, Constants.ShooterConstants.intakeVelocity);
    },(() -> {
        motorTop.set(ControlMode.PercentOutput, 0);
        motorBottom.set(ControlMode.PercentOutput, 0);
        moveArmToPosition(0);
    }));
  }

  public Command moveArmZero() {
    return this.run(() -> moveArmToPosition(-10000));
  }


  public Command resetArm() {
    return this.runOnce(() -> {
        rotateMotor.setSelectedSensorPosition(0);
        rotateMotorFollower.setSelectedSensorPosition(0);

    });
  }
}
