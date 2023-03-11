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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX motorBottom; 
  TalonFX motorTop;
  TalonFX rotateMotor;
  TalonFX rotateMotorFollower;


  /** Creates a new Shooter. */
  public Shooter() {
    motorTop = new TalonFX(Constants.ShooterConstants.motorTopID);
    motorBottom = new TalonFX(Constants.ShooterConstants.motorBottomID);
    rotateMotor = new TalonFX(Constants.ShooterConstants.rotateMotor);
    rotateMotorFollower = new TalonFX(Constants.ShooterConstants.rotateMotorFollower);

    rotateMotorFollower.follow(rotateMotor);
    rotateMotor.setInverted(TalonFXInvertType.Clockwise);
    rotateMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);

    motorTop.setNeutralMode(NeutralMode.Coast);
    motorBottom.setNeutralMode(NeutralMode.Coast);
    

    motorTop.setInverted(TalonFXInvertType.Clockwise);
    motorBottom.setInverted(TalonFXInvertType.Clockwise);

    // encoder.setDistancePerRotation(360);
    rotateMotor.configFactoryDefault();
    resetPos();
    // rotateMotor.config_kF(0, Constants.ArmConstants.UP_kF, 0);
    rotateMotor.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kP, Constants.ShooterConstants.kTimeoutMs);
    rotateMotor.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kI, Constants.ShooterConstants.kTimeoutMs);
    rotateMotor.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kD, Constants.ShooterConstants.kTimeoutMs);

    rotateMotor.configMotionSCurveStrength(2);

    rotateMotor.setInverted(TalonFXInvertType.Clockwise);
    rotateMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);

    rotateMotor.setNeutralMode(NeutralMode.Brake);

    rotateMotorFollower.setNeutralMode(NeutralMode.Brake);

    /* Set acceleration and vcruise velocity - see documentation */
		rotateMotor.configMotionCruiseVelocity(15000, Constants.ShooterConstants.kTimeoutMs);
		rotateMotor.configMotionAcceleration(3000, Constants.ShooterConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		rotateMotor.setSelectedSensorPosition(0, Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kTimeoutMs);

    rotateMotor.configNeutralDeadband(0.0001, Constants.ShooterConstants.kTimeoutMs);
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
      System.out.println("Moving to position");

      int kMeasuredPosHorizontal = 9000; //Position measured when arm is horizontal
      double kTicksPerDegree = (2048 / 360) * 12; //Sensor is 1:1 with arm rotation
      double currentPos = rotateMotor.getSelectedSensorPosition();
      double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
      double radians = java.lang.Math.toRadians(degrees);
      double cosineScalar = java.lang.Math.cos(radians);


      
      double maxGravityFF = -.075;

      /* Test Out Percent Output for arm */
      // if (rotateMotor.getStatorCurrent() > 65) {
      //   rotateMotor.set(ControlMode.PercentOutput, 0);
      // }
      // rotateMotor.set(ControlMode.Position, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar); // Colin 5431 
      rotateMotor.set(ControlMode.Position, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF);
      rotateMotorFollower.follow(rotateMotor);
      System.out.println("Move to: " + targetPos + "Current: " + rotateMotor.getSelectedSensorPosition());  
  }

  private void resetPos() {
    rotateMotor.setSelectedSensorPosition(0, Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kTimeoutMs);
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
