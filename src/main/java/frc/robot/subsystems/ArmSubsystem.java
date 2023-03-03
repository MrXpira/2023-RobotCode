package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX armMotorMaster = new TalonFX(Constants.ArmConstants.armMotorMasterID);
    private final TalonFX armMotorFollower = new TalonFX(Constants.ArmConstants.armMotorFollowerID);
    //private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    public ArmSubsystem() {
       // encoder.setDistancePerRotation(360);
        armMotorMaster.configFactoryDefault();
        armMotorMaster.setSelectedSensorPosition( 0);

        // armMotorMaster.config_kF(0, Constants.ArmConstants.UP_kF, 0);
        armMotorMaster.config_kP(0, Constants.ArmConstants.UP_kP, 0);
        armMotorMaster.config_kI(0, Constants.ArmConstants.UP_kI, 0);
        armMotorMaster.config_kD(0, Constants.ArmConstants.UP_kD, 0);

        // armMotorMaster.config_kF(1, Constants.ArmConstants.DOWN_kF, 0);
        // armMotorMaster.config_kP(1, Constants.ArmConstants.DOWN_kP, 0);
        // armMotorMaster.config_kI(1, Constants.ArmConstants.DOWN_kI, 0);
        // armMotorMaster.config_kD(1, Constants.ArmConstants.DOWN_kD, 0);

        armMotorMaster.configMotionSCurveStrength(2);

        armMotorMaster.setInverted(TalonFXInvertType.CounterClockwise);
        armMotorMaster.setNeutralMode(NeutralMode.Brake);

        armMotorFollower.setNeutralMode(NeutralMode.Brake);
        armMotorFollower.setInverted(TalonFXInvertType.Clockwise);

        // if (encoder.isConnected()) {
        //   System.out.println("Falcon Encoders Set");
        //   armMotorMaster.setSelectedSensorPosition((2048/8192) * encoder.getAbsolutePosition());
        //   armMotorFollower.setSelectedSensorPosition((2048/8192) * encoder.getAbsolutePosition());
        // }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Master Falcon Position", armMotorMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Follower Falcon Position", armMotorFollower.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Follower Falcon Voltage", armMotorFollower.getMotorOutputVoltage());
        SmartDashboard.putNumber("Arm Master Falcon Voltage", armMotorMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("Arm Angle:", getArmCurrentAngleDegrees());
        SmartDashboard.putNumber("Feed Forward", calculateFeedForward());
    }

    public Command resetSensor() {
      return runOnce(
          () -> {
              armMotorMaster.setSelectedSensorPosition(0);
              armMotorFollower.setSelectedSensorPosition(0);
          }
        );
    }

    public Command moveArm(DoubleSupplier firstTrigger, DoubleSupplier secondTrigger) {
      return run(
        () -> {
          double outputValue = firstTrigger.getAsDouble() - secondTrigger.getAsDouble();
          armMotorMaster.set(ControlMode.PercentOutput, outputValue);
          armMotorFollower.follow(armMotorMaster);
          System.out.println("Entered Command: " + outputValue);
        }
      );
    }

    /* Gravity Compensation */
    private double calculateFeedForward() {
      double radians = java.lang.Math.toRadians(getArmCurrentAngleDegrees());
      double cosineScalar = java.lang.Math.cos(radians);

      double maxGravityFF = 0.07;

      return maxGravityFF * cosineScalar;
      
    }

    private double getArmCurrentAngleDegrees() {

      // CHANGE TO REV ENCODER
      int kMeasuredPosHorizontal = 0; //Position measured when arm is horizontal
      double kTicksPerDegree = (2048 / 360) * 40 * (22/18); //Sensor is 48.89 : 1 with arm rotation MOVE TO CONSTANTS FOLDER
      double currentPos = armMotorMaster.getSelectedSensorPosition();
      double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
      return degrees;
    }

    public Command moveArmToPosition(double targetPos) {
      return runOnce(() -> {
        armMotorMaster.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, calculateFeedForward());
        armMotorFollower.follow(armMotorMaster);
        System.out.println("Move to position command ran");
      });
    }
  
    public Command stop() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.PercentOutput, 0);
                armMotorFollower.set(TalonFXControlMode.PercentOutput, 0);
            }
        );
    }
  }