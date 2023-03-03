package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{
    private final int kPIDLoopIdx = 0;
    private final boolean kSensorPhase = true;
    //PID's are subject to change.
    private final double kP = 0.15;
    private final double kI = 0;
    private final double kD = 1.0;
    private final double kF = 0.0;
    private final double kPeakOutput = 0.20;
    private final int kTimeoutMs = 30;

    private static TalonFX m_clawMotor;

    private static double normalCurrent;

    public ClawSubsystem(){
        m_clawMotor = new TalonFX(ClawConstants.kClawMotor);
         normalCurrent = m_clawMotor.getSupplyCurrent();
        configMotor(m_clawMotor);
          
    }

    private void configMotor(TalonFX motor) {
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

        motor.setSensorPhase(kSensorPhase);

        motor.configNominalOutputForward(0, kTimeoutMs);
        motor.configNominalOutputReverse(0, kTimeoutMs);

        motor.configPeakOutputForward(kPeakOutput, kTimeoutMs);
        motor.configPeakOutputReverse(-kPeakOutput, kTimeoutMs);

        motor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

        motor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        motor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        motor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
        motor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);

        motor.setNeutralMode(NeutralMode.Brake);
    }
//Voltages are subject to change
    public Command openClaw(boolean isPressed) {
      return this.run(() -> {
        if (isPressed && m_clawMotor.getStatorCurrent() <= normalCurrent){
          m_clawMotor.set(ControlMode.PercentOutput, -0.3);
        } 
      }); 
    }

    public static void holdBall() {
      
        while(m_clawMotor.getStatorCurrent() <= normalCurrent){
            m_clawMotor.set(ControlMode.PercentOutput, 0.3);
            System.out.println("Current: " + m_clawMotor.getStatorCurrent());
        }
    }
    
    public Command moveClaw(DoubleSupplier outputValue){
        return this.run(() -> {
            m_clawMotor.set(ControlMode.PercentOutput, outputValue.getAsDouble());
        }); 
    };

    public double getClawPosition(){
        return m_clawMotor.getSelectedSensorPosition();
    }

      @Override
      public void periodic() {
          SmartDashboard.putNumber("Claw Encoder Count", getClawPosition());
          SmartDashboard.putNumber("Normal Currrent", normalCurrent);
          SmartDashboard.putNumber("Current", m_clawMotor.getSupplyCurrent());
      }
  }

    
    