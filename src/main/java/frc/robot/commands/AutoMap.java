package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


import java.util.HashMap;

public class AutoMap {
  public final HashMap<String, Command> eventMap = new HashMap<>();
  
  public AutoMap(ShootingArmCommands shootingArmCommands, Shooter shooter, Arm arm, SwerveSubsystem swerveDrive) {
    eventMap.put("Intake", shootingArmCommands.Intake(5));
    eventMap.put("ShootHigh", shooter.shootHigh());
    eventMap.put("ShootLow", shooter.shootLow());
    eventMap.put("ShootMid", shooter.shootMid());
    eventMap.put("Cannon", shooter.shootCannon());
    eventMap.put("Balance", swerveDrive.balanceRobot());
    eventMap.put("RevBalance", swerveDrive.revbalanceRobot());
    // eventMap.put("MoveOntoStation", swerveDrive.moveRevOntoChargeStation());
  }

  public HashMap<String, Command> getEventMap() {
    return eventMap;
  }
}