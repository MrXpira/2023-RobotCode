import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;

import javax.sound.midi.Soundbank;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;



class IntakeTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  Intake m_intake;
  DoubleSolenoidSim m_simPiston;
  CommandScheduler scheduler;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_intake = new Intake(); // create our intake
    m_simPiston =
        new DoubleSolenoidSim(
            PneumaticsModuleType.REVPH,
            IntakeConstants.kPistonFwdChannel,
            IntakeConstants.kPistonRevChannel); // create our simulation solenoid
    scheduler = CommandScheduler.getInstance();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    m_intake.close(); // destroy our intake object
  }

  @Test // marks this method as a test
  void doesntWorkWhenClosed() {
    m_intake.retract(); // close the intake
    m_intake.activate(0.5); // try to activate the motor
    assertEquals(
        0.0, m_intake.getSpeedPercentage(), DELTA); // make sure that the value set to the motor is 0
  }

  @Test
  void worksWhenOpen() {
    m_intake.deploy();
    m_intake.activate(0.3);
    assertEquals(0.3, m_intake.getSpeedPercentage(), DELTA);
  }

  @Test
  void retractTest() {
    m_intake.retract();
    assertEquals(DoubleSolenoid.Value.kReverse, m_simPiston.get());
  }

  @Test
  void deployTest() {
    m_intake.deploy();
    assertEquals(DoubleSolenoid.Value.kForward, m_simPiston.get());
  }

  // @Test
  // void commandDeployTest() {
  //   m_intake.deployIntake(.5).initialize();
  //   CommandScheduler.getInstance().run();
  //   assertEquals(DoubleSolenoid.Value.kForward, m_simPiston.get());
  //   assertEquals(.5, m_intake.getSpeedPercentage(), DELTA);
  // }

  // @Test
  // void commandRetractTest() {
  //   DriverStationSim.setEnabled(true);
  //   m_intake.retractIntake().schedule();
  //   CommandScheduler.getInstance().run();
  //   CommandScheduler.getInstance()
  //   .onCommandInitialize(
  //       command -> System.out.print("Command Init"));
  //   assertEquals(DoubleSolenoid.Value.kReverse, m_simPiston.get());
  //   assertEquals(0, m_intake.getSpeedPercentage(), DELTA);

  // }
}