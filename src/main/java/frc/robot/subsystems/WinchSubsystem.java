package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class WinchSubsystem extends SubsystemBase
{
    private final int kPIDLoopIdx = 0;
    private final boolean kSensorPhase = true;
    //PID's are subject to change.
    private static double kP = 0.15;
    private static double kI = 0;
    private static double kD = 1.0;
    private static double kF = 0.0;
    private final double kPeakOutput = 0.20;
    private final int kTimeoutMs = 30;

    private static TalonFX m_winchMotor;



    private static double normalCurrent;




    public WinchSubsystem(){
        m_winchMotor = new TalonFX(Constants.WinchConstants.kwinchMotor);
         normalCurrent = m_winchMotor.getSupplyCurrent();
        configMotor(m_winchMotor);
         
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


    public Command moveWinch(double outputValue)
    {
        return this.run(()-> {
            
            m_winchMotor.set(ControlMode.PercentOutput, outputValue);
            if (m_winchMotor.getStatorCurrent() < normalCurrent - 0.05){
                m_winchMotor.set(ControlMode.Current, normalCurrent - m_winchMotor.getStatorCurrent());
            }
            
            
        }      
        );
    }
};    



