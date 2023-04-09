package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  WPI_TalonFX shootMotor; 
  WPI_TalonFX shootMotorFollower;

  DoubleLogEntry shooterSpeed;
  DoubleLogEntry shooterCurrent;
  double pValue;
  public enum ShootSpeed {
    Stop,
    High,
    Mid,
    Low,
    Intake,
    Cannon,
    CannonLow
  } 
  /** Creates a new Shooter. */
  public Shooter() {
    shootMotorFollower = new WPI_TalonFX(ShooterConstants.SHOOTER_TOP_MOTOR, Constants.CANBUS);
    shootMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_BOTTOM_MOTOR, Constants.CANBUS);

    shootMotor.set(ControlMode.PercentOutput, 0);
    shootMotorFollower.set(ControlMode.PercentOutput, 0);

    // pValue = Shuffleboard.getTab("Shoot").addPersistent("PValue", 0).getEntry().getDouble(0);
    configMotors();
    startLogging();

    
  }

  private void configMotors() {
    /* Config Intake Motors */
    shootMotor.setInverted(TalonFXInvertType.Clockwise);
    shootMotor.setNeutralMode(NeutralMode.Coast);
    shootMotor.configVoltageCompSaturation(10);
    shootMotor.enableVoltageCompensation(true);

    shootMotorFollower.setInverted(TalonFXInvertType.Clockwise);
    shootMotorFollower.setNeutralMode(NeutralMode.Coast);
    shootMotorFollower.configVoltageCompSaturation(10); // "full output" will now scale to 11 Volts for all control modes when enabled.
    shootMotorFollower.enableVoltageCompensation(true);
  }

  private void startLogging() {
    DataLog log = DataLogManager.getLog();

    shooterSpeed = new DoubleLogEntry(log, "/shooter/speed");
    shooterCurrent = new DoubleLogEntry(log, "/shooter/current");
  }

  @Override
  public void periodic() {
    shooterSpeed.append(shootMotor.getSelectedSensorPosition());
    shooterCurrent.append(shootMotor.getStatorCurrent());
    SmartDashboard.putNumber("StatorCurrentShooterTopIntake", shootMotorFollower.getStatorCurrent());
    // System.out.println(Shuffleboard.getTab("Shoot").addPersistent("PValue", 0).getEntry().getDouble(0));
  }

  private void shoot(ShootSpeed shootSpeed) {
    double speedTop;
    double speedBottom;
    switch (shootSpeed) {
      case Low: 
        speedTop = Constants.ShooterConstants.bottomGoalVelocityTopMotor;
        speedBottom = Constants.ShooterConstants.bottomGoalVelocityBottomMotor;
        break;
      case Mid: 
        speedTop = Constants.ShooterConstants.midGoalVelocityTopMotor;
        speedBottom = Constants.ShooterConstants.midGoalVelocityBottomMotor;
        break;
      case High: 
        speedTop = Constants.ShooterConstants.highGoalVelocityTopMotor;
        speedBottom = Constants.ShooterConstants.highGoalVelocityBottomMotor;
        break;
      case Intake: 
        speedTop = -Constants.ShooterConstants.intakeVelocity;
        speedBottom = -Constants.ShooterConstants.intakeVelocity;
        break;
      case Cannon:
        speedTop = Constants.ShooterConstants.cannonGoalVelocityTopMotor;
        speedBottom = Constants.ShooterConstants.cannonGoalVelocityBottomMotor;
        break;
        case CannonLow:
        speedTop = Constants.ShooterConstants.cannon2GoalVelocityTopMotor;
        speedBottom = Constants.ShooterConstants.cannon2GoalVelocityBottomMotor;
        break;
      default: 
        speedTop = 0;
        speedBottom = 0;
        break;
    }
    
    shootMotorFollower.set(ControlMode.PercentOutput, speedTop);
    shootMotor.set(ControlMode.PercentOutput, speedBottom);
  }

  public Command shootHigh() {
    return this.runOnce(() -> {
      shoot(ShootSpeed.High);
    }).andThen(new WaitCommand(ShooterConstants.shootWaitTime)).andThen(() -> {
    shootMotorFollower.set(ControlMode.PercentOutput, 0);
    shootMotor.set(ControlMode.PercentOutput, 0);
    });
  }

  public Command shootMid() {
    return this.runOnce(() -> {
      shoot(ShootSpeed.Mid);
    }).andThen(new WaitCommand(ShooterConstants.shootWaitTime)).andThen(() -> {
    shootMotorFollower.set(ControlMode.PercentOutput, 0);
    shootMotor.set(ControlMode.PercentOutput, 0);
    });
  }

  public Command shootLow() {
    return this.runOnce(() -> {
      shoot(ShootSpeed.Low);
    }).andThen(new WaitCommand(ShooterConstants.shootWaitTime)).andThen(() -> {
    shootMotorFollower.set(ControlMode.PercentOutput, 0);
    shootMotor.set(ControlMode.PercentOutput, 0);
    });
  }

  public Command shootCannon() {
    return this.runOnce(() -> {
      shoot(ShootSpeed.Cannon);
    }).andThen(new WaitCommand(ShooterConstants.shootWaitTime)).andThen(() -> {
    shootMotorFollower.set(ControlMode.PercentOutput, 0);
    shootMotor.set(ControlMode.PercentOutput, 0);
    });
  }

  public Command shootFastHigh() {
    return this.runOnce(() -> {
      shoot(ShootSpeed.CannonLow);
    }).andThen(new WaitCommand(ShooterConstants.shootWaitTime)).andThen(() -> {
    shootMotorFollower.set(ControlMode.PercentOutput, 0);
    shootMotor.set(ControlMode.PercentOutput, 0);
    });
  }

  public Command stop() {
    return this.runOnce(() -> {
      shootMotorFollower.set(ControlMode.PercentOutput, 0);
      shootMotor.set(ControlMode.PercentOutput, 0);
    });
  }

  public Command intake() {
    return this.runEnd(() -> shoot(ShootSpeed.Intake), 
                      (() -> shoot(ShootSpeed.Stop)));
  }

  public Command intake(DoubleSupplier intakeSpeed) {
    return this.run(() -> {
                      shootMotor.set(ControlMode.PercentOutput, -intakeSpeed.getAsDouble());
// 
                      //  shootMotorFollower.set(ControlMode.PercentOutput, -intakeSpeed.getAsDouble());
                    });
  }

  public Command intakeTime(int seconds) {
    return this.runOnce(() -> {
      shoot(ShootSpeed.Intake);
    }).andThen(new WaitCommand(seconds)).andThen(() -> shoot(ShootSpeed.Stop));
  }

  public Command intakeCurrent() {
    return this.run(() -> {
      shoot(ShootSpeed.Intake);
    }).andThen(new WaitCommand(1)).andThen(() -> shoot(ShootSpeed.Intake)).until(() -> shootMotor.getStatorCurrent() > Constants.ShooterConstants.currentThreshold).andThen(() -> shoot(ShootSpeed.Stop));
  }
}
