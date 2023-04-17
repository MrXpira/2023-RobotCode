package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Shooter.ShootSpeed;

public class ShootingArmCommands {

    private Shooter shooter;

    private Arm arm;

    public ShootingArmCommands(Shooter shooter, Arm arm) {
      this.shooter = shooter;
      this.arm = arm;

    }

    public Command Intake() {
      return Commands.parallel(arm.moveArmToPosition(ArmPosition.Intake), shooter.intake());
    }

    public Command Intake(double time) {
      return Commands.parallel(
        arm.moveArmToPosition(ArmPosition.Intake),
        shooter.intake()).withTimeout(time).andThen(Rest());
    }
        
    public Command Rest() {
      return Commands.parallel(arm.moveArmToPosition(ArmPosition.Rest), shooter.stop());
    }

    public Command ShootHigh() {
      return Commands.sequence(arm.moveArmToPosition(ArmPosition.High), shooter.shootHigh());
    } 

    public Command ShootMid() {
      return Commands.sequence(arm.moveArmToPosition(ArmPosition.Mid), shooter.shootMid());
    }

    public Command ShootLow() {
      return Commands.sequence(arm.moveArmToPosition(ArmPosition.Low), shooter.shootLow());
    }

    public Command Cannon() {
      return Commands.sequence(arm.moveArmToPosition(ArmPosition.Cannon), shooter.shootCannon());
    }

    public Command Shoot() {
      System.out.println(arm.getCurrentPosition());
      switch (arm.getCurrentPosition()) {
        case High: 
          return shooter.shootHigh();
        case Low: 
          return shooter.shootLow();
        case Mid: 
          return shooter.shootMid();
        case Cannon:
          return shooter.shootCannon();
        default:
          return shooter.stop();
      }
    }
}