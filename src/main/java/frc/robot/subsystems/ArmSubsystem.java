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
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    public ArmSubsystem() {
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

        if (encoder.isConnected()) {
          encoder.getAbsolutePosition();
        }
       

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Master Falcon Position", armMotorMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Follower Falcon Position", armMotorFollower.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Follower Falcon Voltage", armMotorFollower.getMotorOutputVoltage());
        SmartDashboard.putNumber("Arm Master Falcon Voltage", armMotorMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("Arm Angle:", getArmCurrentAngleDegrees());
        SmartDashboard.putBoolean("Encoder Connected", encoder.isConnected());
        SmartDashboard.putNumber("Encoder Angle", encoder.getAbsolutePosition());
    }

    public Command resetSensor() {
      return runOnce(
          () -> {
              armMotorMaster.setSelectedSensorPosition(0);
              armMotorFollower.setSelectedSensorPosition(0);
          }
        );
    }

    public Command moveArm(double outputValue) {
      return run(
        () -> {
          armMotorMaster.set(ControlMode.PercentOutput, outputValue);
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
      int kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal
      double kTicksPerDegree = (2048 / 360); //Sensor is 48.89 : 1 with arm rotation MOVE TO CONSTANTS FOLDER
      double currentPos = armMotorMaster.getSelectedSensorPosition();
      double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
      return degrees;
    }

    public Command moveArmToPosition(double targetPos) {
      return runOnce(() -> armMotorMaster.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, calculateFeedForward()));
    }

    
  
    public Command stop() {
        return runOnce(
            () -> {
                armMotorMaster.set(TalonFXControlMode.PercentOutput, 0);
                armMotorFollower.set(TalonFXControlMode.PercentOutput, 0);
            }
        );
    }



    // public Command setPosition(double position) {
    //     return runOnce(
    //         () -> {
    //                 //manageMotion(position);
    //                 moveArmToPosition();
    //                 armMotorFollower.follow(armMotorMaster);
    //                 armMotorFollower.setInverted(InvertType.FollowMaster);
    //         }
    //     );
    // }

//     public Command setVoltage(float voltage) {
//         return runOnce(
//             () -> {
//                 armMotorMaster.set(ControlMode.PercentOutput, voltage);
//                 armMotorFollower.follow(armMotorMaster);
//                 armMotorFollower.setInverted(InvertType.FollowMaster);
//             }
//         );
//     }
    
//     public void manageMotion(double targetPosition) {
//         double currentPosition = armMotorMaster.getSelectedSensorPosition();
    
//         // going up
//         if(currentPosition < targetPosition) {
    
//           // set accel and velocity for going up
//           armMotorMaster.configMotionAcceleration(Constants.ArmConstants.CRUISE_VELOCITY_ACCEL_UP, 0);
//           armMotorMaster.configMotionCruiseVelocity(Constants.ArmConstants.CRUISE_VELOCITY_ACCEL_UP, 0);
    
//           // select the up gains
//           armMotorMaster.selectProfileSlot(0, 0);
//           SmartDashboard.putBoolean("Going Up or Down", true);
    
//         } else {
          
//           // set accel and velocity for going down
//           armMotorMaster.configMotionAcceleration(Constants.ArmConstants.CRUISE_VELOCITY_ACCEL_DOWN, 0);
//           armMotorMaster.configMotionCruiseVelocity(Constants.ArmConstants.CRUISE_VELOCITY_ACCEL_DOWN, 0);
    
//           // select the down gains
//           armMotorMaster.selectProfileSlot(1, 0);
//           SmartDashboard.putBoolean("Going Up or Down", false);

//         }
    
//       }
}
