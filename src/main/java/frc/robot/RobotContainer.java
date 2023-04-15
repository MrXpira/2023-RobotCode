package frc.robot;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.swervedrive.auto.ExampleAuto;
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
    // private final CANdleSubsystem candleSubsystem = new CANdleSubsystem();

    /* Commands */
    private final ShootingArmCommands shootingArmCommands = new ShootingArmCommands(shooter, arm);
    private final AutoMap autoMap = new AutoMap(shootingArmCommands, shooter, arm, s_Swerve);
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

        arm.setDefaultCommand(shootingArmCommands.Rest());
        shooter.setDefaultCommand(shooter.stop());

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


        operatorXbox.x().whileTrue(shootingArmCommands.Intake());
        operatorXbox.rightBumper().whileTrue(arm.moveArmToPosition(ArmPosition.Cannon));
        operatorXbox.b().onTrue(shooter.shootMid());
        operatorXbox.y().onTrue(shooter.shootFastHigh());
        operatorXbox.a().onTrue(shooter.shootLow());
        operatorXbox.leftBumper().onTrue(shooter.shootCannon());
        operatorXbox.back().onTrue(shooter.shootHigh());
        operatorXbox.leftStick().whileTrue(arm.moveArmToPosition(ArmPosition.LowStack));

        /* Manual Override */
        // operatorXbox.start().whileTrue(arm.moveArm(() -> operatorXbox.getRawAxis(3)-operatorXbox.getRawAxis(2)));
        operatorXbox.rightStick().whileTrue(shooter.intake(() -> operatorXbox.getRawAxis(3) * .2));
    }

    private void initializeChooser() {
  
        chooser.setDefaultOption(
              "Shoot High And Balance - No PathPlanner",
              shooter.shootHigh().andThen(
              s_Swerve.moveOntoChargeStation())
              .andThen(s_Swerve.balanceRobot()));

        chooser.addOption(
        "Calibration 2 Meter",
                builder.getSwerveCommand(
                    PathPlanner.loadPathGroup(
                        "Test Path", new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION))));

        chooser.addOption(
            "Just Shoot", 
            shooter.shootHigh());

        // chooser.addOption(
        //     "Shoot High And Taxi-Balance - No PathPlanner",
        //     shooter.shootHigh().andThen(
        //     s_Swerve.moveOntoChargeStation())
        //     .andThen(s_Swerve.move()).withTimeout(4.5)
        //     .andThen(s_Swerve.moveRevOntoChargeStation())
        //     .andThen(s_Swerve.revbalanceRobot()));
    
        SmartDashboard.putData("Auto", chooser);
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return chooser.getSelected();
        // return new ExampleAuto(s_Swerve);
        return builder.getSwerveCommand(PathPlanner.loadPathGroup("New Path", new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION)));
    }


}


// TODO //

// Test intake command
// Test balance without pathplanner 
// test path planner auto 
// make sure everything else works 