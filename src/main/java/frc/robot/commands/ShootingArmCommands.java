package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmPosition;

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

    public Command Cannon() {
      return arm.moveArmToPosition(ArmPosition.Cannon).withTimeout(.2).andThen(shooter.shootCannon().andThen(Rest()));
    }
}