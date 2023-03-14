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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter shooter = new Shooter();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    //         s_Swerve::getPose, // Pose2d supplier
    //         s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    //         Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
    //         new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    //         new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    //         s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    //         Constants.eventMap,
    //         true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //         s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    //     );

    /* Driver Buttons */
    private final JoystickButton d_Y = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton d_A = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton d_X = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton d_B = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton d_leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton d_rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final POVButton d_povRight = new POVButton(driver, 90);
    private final POVButton d_povDown = new POVButton(driver, 180);
    private final POVButton d_povLeft = new POVButton(driver, 270);

    /* Operator Buttons */
    private final JoystickButton o_rightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton o_rightStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton o_leftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton o_leftStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton o_start = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton o_back = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton o_A = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton o_B = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton o_X = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton o_Y = new JoystickButton(operator, XboxController.Button.kY.value);
    private final POVButton o_povUp = new POVButton(operator, 0);
    private final POVButton o_povRight = new POVButton(operator, 90);
    private final POVButton o_povDown = new POVButton(operator, 180);
    private final POVButton o_povLeft = new POVButton(operator, 270);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> d_leftBumper.getAsBoolean()
            )
        );

        shooter.setDefaultCommand(shooter.moveArm(() -> (operator.getRawAxis(3) - operator.getRawAxis(2)*.3)));

        // https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html#setting-up-sendablechooser
        // Configure the button bindings
        configureBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        /* Driver Buttons */
        d_Y.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        d_A.whileTrue(s_Swerve.lockWheels());
        d_B.whileTrue(s_Swerve.moveOntoChargeStation());
        d_X.whileTrue(s_Swerve.balanceRobot());



        // o_rightBumper.whileTrue(shooter.moveArmToPosition(1500));
        // o_leftBumper.whileTrue(shooter.shoot(-.3,-.3)).whileFalse(shooter.stopShooter());
        // o_rightStick.whileTrue(shooter.stopShooter());

        // o_leftStick.whileTrue(shooter.resetPos());
        // o_A.whileTrue(shooter.moveArmToPosition(6000));
        o_Y.whileTrue(shooter.shoot(Constants.ShooterConstants.highGoalVelocityTopMotor, Constants.ShooterConstants.highGoalVelocityBottomMotor)); // Shoot high
        o_A.whileTrue(shooter.shoot(Constants.ShooterConstants.bottomGoalVelocityTopMotor, Constants.ShooterConstants.bottomGoalVelocityBottomMotor)); // Shoot low
        o_B.whileTrue(shooter.shoot(Constants.ShooterConstants.midGoalVelocityTopMotor, Constants.ShooterConstants.midGoalVelocityBottomMotor)); // Shoot medium
        o_X.whileTrue(shooter.intake());
        //o_rightBumper.whileTrue(shooter.moveArmZero());
        o_leftBumper.whileTrue(shooter.resetArm());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        /* Just Shoot High Command */
        // shooter.shoot(Constants.ShooterConstants.highGoalVelocityTopMotor, Constants.ShooterConstants.highGoalVelocityBottomMotor).withTimeout(1.5);

        /* Just Shoot Mid Command */
        //return shooter.shoot(Constants.ShooterConstants.midGoalVelocityTopMotor, Constants.ShooterConstants.midGoalVelocityBottomMotor).withTimeout(1.5);

        /* Shoot High And Balance Without Pathplanner */
        return shooter.shoot(Constants.ShooterConstants.highGoalVelocityTopMotor, Constants.ShooterConstants.highGoalVelocityBottomMotor).withTimeout(1.5).andThen(s_Swerve.moveRevOntoChargeStation()).andThen(s_Swerve.reverseBalance());

        /* Shoot High And Balance */
        //return shooter.shoot(Constants.ShooterConstants.highGoalVelocityTopMotor, Constants.ShooterConstants.highGoalVelocityBottomMotor).withTimeout(1.5).andThen(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true ).andThen(s_Swerve.balanceRobot())); //SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true ),s_Swerve.balanceRobot());

        /* Shoot Mid And Balance */
        //return shooter.shoot(Constants.ShooterConstants.midGoalVelocityTopMotor, Constants.ShooterConstants.midGoalVelocityBottomMotor).withTimeout(1.5).andThen(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("ShootBalance", new PathConstraints(4, 3)), true ).andThen(s_Swerve.balanceRobot())); //SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true ),s_Swerve.balanceRobot());

        
        /* Shoot And Grab */
        //return shooter.shoot(Constants.ShooterConstants.highGoalVelocityTopMotor, Constants.ShooterConstants.highGoalVelocityBottomMotor).withTimeout(1.5).andThen(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("Advance Auto", new PathConstraints(4, 3)), true )).andThen(shooter.intake()).withTimeout(1.4);

        /* Shoot And Grab And Balance */
        //return shooter.shoot(Constants.ShooterConstants.highGoalVelocityTopMotor, Constants.ShooterConstants.highGoalVelocityBottomMotor).withTimeout(1.5).andThen(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("Advance Auto", new PathConstraints(4, 3)), true )).andThen(shooter.intake()).withTimeout(1.4).andThen(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("Advance Auto 2", new PathConstraints(4, 3)), false)).andThen(s_Swerve.balanceRobot()); //SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true ),s_Swerve.balanceRobot());

        /* Shoot, Grab, And Shoot */
        //return new EventAutoTest(s_Swerve, shooter);

        //return s_Swerve.balanceRobot(); //SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true ),s_Swerve.balanceRobot());
        //return s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true ).andThen(s_Swerve.balanceRobot()); //SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true )
    }


}


// TODO //

// Test intake command
// Test balance without pathplanner 
// test path planner auto 
// make sure everything else works 