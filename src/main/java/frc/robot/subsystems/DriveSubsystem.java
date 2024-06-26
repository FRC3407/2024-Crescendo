// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // Using NavX

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private Field2d m_field = new Field2d();

  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_rotationCommanded = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public double wait_seconds;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getHeading(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field", m_field);
    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Constants.ModuleConstants.kPPDrivingP,
                Constants.ModuleConstants.kPPDrivingI, Constants.ModuleConstants.kPPDrivingD), // Translation PID
                                                                                               // constants
            new PIDConstants(Constants.ModuleConstants.kPPTurningP,
                Constants.ModuleConstants.kPPTurningI, Constants.ModuleConstants.kPPTurningD), // Rotation PID constants
            Constants.DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Constants.DriveConstants.kBaseRadius, // Drive base radius in metbers. Distance from robot center to
                                                  // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
    // Load the path we want to pathfind to and follow
    // PathPlannerPath path = PathPlannerPath.fromPathFile("T1");

    // // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    // PathConstraints constraints = new PathConstraints(
    //     3.0, 4.0,
    //     Units.degreesToRadians(540), Units.degreesToRadians(720));

    // // Since AutoBuilder is configured, we can use it to build pathfinding commands
    // Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
    //     path,
    //     constraints,
    //     3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    // );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_field.setRobotPose(m_odometry.update(
        getHeading(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }));
    SmartDashboard.putNumber("X in meters", getPose().getX());
    SmartDashboard.putNumber("Y in meters", getPose().getY());
    SmartDashboard.putNumber("Gyro Angle", getHeading().getDegrees());
    SmartDashboard.putNumber("Gyro Fused Yaw", m_gyro.getFusedHeading());
    SmartDashboard.putNumber("Gyro Rate", m_gyro.getRate());
    SmartDashboard.putNumber("Gyro Compass Heading", m_gyro.getCompassHeading());
    SmartDashboard.putNumber("Gyro Delta", m_gyro.getCompassHeading() - getHeading().getDegrees());
    // SmartDashboard.putString("Selected Auto",
    // Controls.getSelectedAutoCommand().getName());
    SmartDashboard.putNumber("rot", m_rotationCommanded);
    SmartDashboard.putNumber("Velocity", Math.sqrt(
        Math.pow(getChassisSpeeds().vxMetersPerSecond, 2.0) + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2.0)));
    SmartDashboard.putNumber("Front Left Velocity", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Velocity", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Rear Left Velocity", m_rearLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Rear Right Velocity", m_rearRight.getState().speedMetersPerSecond);
    wait_seconds = SmartDashboard.getNumber("Auto Wait Seconds", 0.0);
    SmartDashboard.putNumber("Auto Wait Seconds", wait_seconds);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getHeading(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  Rotation2d intendedRotation = new Rotation2d(getHeading().getDegrees());
  private static long timeOfLastLoop = System.currentTimeMillis();
  double integral = 0.0;
  double Kp = 6;
  double ki = 0;
  double kd = 0;
  double error = 0.0;
  double prevError = 0.0;

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    final boolean isStopped = xSpeed == 0 && ySpeed == 0;
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = isStopped ? m_currentTranslationDir : Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_rotationCommanded = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_rotationCommanded = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = 0.0;
    if (DriverStation.isTeleopEnabled()) {
      intendedRotation = intendedRotation
          .rotateBy(new Rotation2d((m_rotationCommanded) * ((System.currentTimeMillis() - timeOfLastLoop) / 1000.0)
              * DriveConstants.kMaxAngularSpeed));
      timeOfLastLoop = System.currentTimeMillis();
      error = intendedRotation.minus(getHeading()).getRadians(); // Calculate error
      integral+=error;
      rotDelivered = (error * Kp) + (ki*integral)+(kd*(error-prevError)); // Error times P = what to move by
      prevError = error;
    } else {
      rotDelivered = m_rotationCommanded * DriveConstants.kMaxAngularSpeed;
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("Auto Wait Seconds", 0.0);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    intendedRotation = new Rotation2d(0);
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Gets the ChassisSpeeds of the robot.
   */
  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassis_speed = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());

    return chassis_speed;
  }

  /**
   * Method that will drive the robot Robot Relative.
   */

  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond/Constants.DriveConstants.kMaxSpeedMetersPerSecond, speeds.vyMetersPerSecond/Constants.DriveConstants.kMaxSpeedMetersPerSecond, speeds.omegaRadiansPerSecond,false,false);
  }
}
