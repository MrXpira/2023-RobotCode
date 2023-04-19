package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


import java.util.HashMap;

public class AutoMap {
  public final HashMap<String, Command> eventMap = new HashMap<>();
  
  public AutoMap(ShootingArmCommands shootingArmCommands, Shooter shooter, Arm arm, SwerveSubsystem swerveDrive, CANdleSubsystem candleSubsystem) {
    eventMap.put("Intake", shootingArmCommands.Intake(1.8));
    eventMap.put("ShootHigh", shooter.shootHigh().alongWith(candleSubsystem.shootingLightsFlash(.7)));
    eventMap.put("ShootLow", shooter.shootLow().alongWith(candleSubsystem.shootingLightsFlash(.1)));
    eventMap.put("ShootMid", shooter.shootMid().alongWith(candleSubsystem.shootingLightsFlash(.4)));
    eventMap.put("Cannon", shooter.shootCannon().alongWith(candleSubsystem.cannonLights()));

  }

  public HashMap<String, Command> getEventMap() {
    return eventMap;
  }
}