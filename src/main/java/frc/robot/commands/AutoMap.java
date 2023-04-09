package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.HashMap;

public class AutoMap {
  public final HashMap<String, Command> eventMap = new HashMap<>();
  
  public AutoMap(SwerveSubsystem s_Swerve, ShootingArmCommands shootingArmCommands) {
    eventMap.put("Intake", shootingArmCommands.Intake());
    // eventMap.put("ShootHigh", shootingArmCommands.ShootHigh());
    // eventMap.put("ShootLow", shootingArmCommands.ShootLow());
    // eventMap.put("ShootMid", shootingArmCommands.ShootMid());
    // eventMap.put("Cannon", shootingArmCommands.Cannon());

  }

  public HashMap<String, Command> getEventMap() {
    return eventMap;
  }
}