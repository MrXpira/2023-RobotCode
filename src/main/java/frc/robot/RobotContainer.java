package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);
    
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final Vision vision = new Vision();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final WinchSubsystem winchSubsystem = new WinchSubsystem();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

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

        armSubsystem.setDefaultCommand(
            armSubsystem.moveArm(
                () -> operator.getRawAxis(3) * .22,
                () -> operator.getRawAxis(2) * .1
            )
        );



        clawSubsystem.setDefaultCommand(
            clawSubsystem.moveClaw(
                () -> operator.getRawAxis(0)
                
            )
        );

        winchSubsystem.setDefaultCommand(winchSubsystem.moveWinch(
                () -> operator.getRawAxis(5)
            )
        );
        
        //winchSubsystem.resetWinchPositionCommand();

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
        d_A.whileTrue(s_Swerve.balanceRobot());
        
        //o_X.onTrue(winchSubsystem.resetWinchPosition());

        //o_leftBumper.onTrue(winchSubsystem.moveWinchToPosition( 120000));

        //o_A.whileTrue(armSubsystem.moveArmToPosition(30000));
        //o_Y.onTrue(clawSubsystem.moveClawPercent(.5));
        //o_Y.onTrue(new SequentialCommandGroup(winchSubsystem.moveWinchToPosition(0), armSubsystem.moveArmToPosition(0)));
        //o_B.onTrue(clawSubsystem.moveClawToPosition(1000));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        return s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true ).andThen(s_Swerve.balanceRobot()); //SequentialCommandGroup(s_Swerve.followTrajectoryCommand(PathPlanner.loadPath("SimpleAuto", new PathConstraints(4, 3)), true ),s_Swerve.balanceRobot());
        //return new PathwithStops(s_Swerve,armSubsystem, vision);
    }

    
}
