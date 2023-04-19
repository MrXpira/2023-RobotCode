package frc.robot;

import java.net.CacheRequest;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.swervedrive.auto.PathBuilder;
import frc.robot.commands.swervedrive.drivebase.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmPosition;
// import frc.robot.subsystems.swervedrive.PoseEstimator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class RobotContainer {
    private SendableChooser<Command> chooser = new SendableChooser<>();

    /* Controllers */
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);
    
    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    private final Shooter shooter = new Shooter();
    private final Arm arm = new Arm();
    private final CANdleSubsystem candleSubsystem = new CANdleSubsystem();

    /* Commands */
    private final ShootingArmCommands shootingArmCommands = new ShootingArmCommands(shooter, arm);
    private final AutoMap autoMap = new AutoMap(shootingArmCommands, shooter, arm, s_Swerve, candleSubsystem);
    private final PathBuilder builder = new PathBuilder(s_Swerve, autoMap.getEventMap());
      

    /* Drive Controls */

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    // private final DigitalInput resetArmSwitch = new DigitalInput(0); // Limit switch on DIO 0
    // private final DigitalInput unlockArmSwitch = new DigitalInput(1); // Limit switch on DIO 1

    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driverXbox.getRawAxis(translationAxis), 
                () -> driverXbox.getRawAxis(strafeAxis), 
                () -> driverXbox.getRawAxis(rotationAxis), 
                () -> driverXbox.leftBumper().getAsBoolean()
            )
        );

        // arm.setDefaultCommand(shootingArmCommands.Rest());
        // shooter.setDefaultCommand(shooter.stop());
        candleSubsystem.setDefaultCommand(candleSubsystem.idleLED());

        configureBindings();
        initializeChooser();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        /* Driver Buttons */
        driverXbox.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driverXbox.a().whileTrue(s_Swerve.lockWheels());
        driverXbox.b().whileTrue(s_Swerve.moveOntoChargeStation());
        driverXbox.x().whileTrue(s_Swerve.balanceRobot());

        /* Arm Movement */
        operatorXbox.x().whileTrue(Commands.parallel(shootingArmCommands.Intake(), candleSubsystem.intake())).whileFalse(shootingArmCommands.Rest());
        operatorXbox.rightBumper().whileTrue(arm.moveArmToPosition(ArmPosition.Cannon)).whileFalse(shootingArmCommands.Rest());
        operatorXbox.leftStick().whileTrue(arm.moveArmToPosition(ArmPosition.LowStack)).whileFalse(shootingArmCommands.Rest());

        // operatorXbox.x().onTrue(shootingArmCommands.Intake(2));

        /* Shooting */
        operatorXbox.b().onTrue(Commands.parallel(shooter.shootMid(), candleSubsystem.shootingLightsFlash(.4)));
        operatorXbox.y().onTrue(Commands.parallel(shooter.shootFastHigh(), candleSubsystem.shootingLightsFlash(.7)));
        operatorXbox.a().onTrue(Commands.parallel(shooter.shootLow(), candleSubsystem.shootingLightsFlash(.1)));
        operatorXbox.leftBumper().onTrue(Commands.parallel(shooter.shootCannon(), candleSubsystem.cannonLights()));
        operatorXbox.back().onTrue(shooter.shootHigh());

        // operatorXbox.back().onTrue(shootingArmCommands.Cannon());

        /* Manual Override */
        // operatorXbox.start().whileTrue(arm.moveArm(() -> operatorXbox.getRawAxis(3)-operatorXbox.getRawAxis(2)));
        operatorXbox.rightStick().whileTrue(shooter.intake(() -> operatorXbox.getRawAxis(3) * .2));
    }

    private void initializeChooser() {
  
        

        chooser.setDefaultOption(
            "No Bump: 3 + Mobility",
            builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "3 Cube Auto", 
                new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION))));
    
        chooser.addOption(
            "No Bump: 2.5 + Mobility + Balance",
            builder.getSwerveCommand(
                PathPlanner.loadPathGroup(
                    "2.5 Cube Balance", 
                    new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION)))
                    .andThen(s_Swerve.moveOntoChargeStation()).andThen(s_Swerve.balanceRobot()));

        chooser.addOption(
            "No Bump: 3 + Mobility + Balance",
            builder.getSwerveCommand(
                PathPlanner.loadPathGroup(
                    "3 Cube Balance", 
                    new PathConstraints(6, 3)))
                    .andThen(s_Swerve.moveRevOntoChargeStation().andThen(s_Swerve.balanceRobot()).andThen(shooter.shootMid())));

        chooser.addOption(
            "Middle: 1 + Mobility + Balance",
            builder.getSwerveCommand(
                PathPlanner.loadPathGroup(
                    "1 High + Mobility + Balance", 
                    new PathConstraints(.5, .5)))
                    .andThen(s_Swerve.moveOntoChargeStation().andThen(s_Swerve.balanceRobot())));

        
        chooser.addOption(
            "Bump Side: 3 + Mobility",
            builder.getSwerveCommand(
                PathPlanner.loadPathGroup(
                    "3 Cube Auto Bump Side", 
                    new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION))));
                
        chooser.addOption(
              "Shoot High And Balance - No PathPlanner",
              shooter.shootHigh().andThen(
              s_Swerve.moveOntoChargeStation())
              .andThen(s_Swerve.balanceRobot()));

    
        SmartDashboard.putData("Auto", chooser);
      }

      public Command getAutoChooserResult() {
        return Commands.runOnce(() -> candleSubsystem.warningLights()).unless(() -> chooser.getSelected() == null);
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }


}


// TODO //

// Test intake command
// Test balance without pathplanner 
// test path planner auto 
// make sure everything else works 