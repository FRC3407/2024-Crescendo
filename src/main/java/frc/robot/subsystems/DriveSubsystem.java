// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
  private SwerveDrivePoseEstimator posEstimate;
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics();

  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

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
          Constants.ModuleConstants.kPPDrivingI, Constants.ModuleConstants.kPPDrivingD), // Translation PID constants
        new PIDConstants(Constants.ModuleConstants.kPPTurningP, 
          Constants.ModuleConstants.kPPTurningI, Constants.ModuleConstants.kPPTurningD), // Rotation PID constants
        Constants.DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
        Constants.DriveConstants.kBaseRadius, // Drive base radius in metbers. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
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
    PathPlannerPath path = PathPlannerPath.fromPathFile("T1");

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints,
        3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    posEstimate = new SwerveDrivePoseEstimator(
      m_kinematics,
      getHeading(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition(),
    },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
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
    // SmartDashboard.putString("Selected Auto", Controls.getSelectedAutoCommand().getName());
    SmartDashboard.putNumber("rot", m_currentRotation);
    SmartDashboard.putNumber("Velocity", Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond, 2.0) + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2.0)));
    SmartDashboard.putNumber("Front Left Velocity", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Velocity", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Rear Left Velocity", m_rearLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Rear Right Velocity", m_rearRight.getState().speedMetersPerSecond);
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
    final boolean isStopped = Math.abs(xSpeed) <= (OIConstants.kDriveDeadband / 2.0)
            && Math.abs(ySpeed) <= (OIConstants.kDriveDeadband / 2.0);

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
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

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
      m_rearRight.getState()
      );
      
    return chassis_speed;
  }

  /**
   * Method that will drive the robot Robot Relative.
   */
  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,false,false);
  }

      Pose3d[] AprilTagList = { //AprilTag field locations (in meters)
            new Pose3d(15.079472, .245872, 1.355852, new Rotation3d(0, 0, 2.094395102)), // 1
            new Pose3d(16.185134, .883666, 1.355852, new Rotation3d(0, 0, 2.094395102)), // 2
            new Pose3d(16.579342, 4.982718, 1.451102, new Rotation3d(0, 0, 3.141492654)), // 3
            new Pose3d(16.579342, 5.547868, 1.451102, new Rotation3d(0, 0, 3.141492654)), // 4
            new Pose3d(14.700758, 8.2042, 1.355852, new Rotation3d(0, 0, 4.71238898)), // 5
            new Pose3d(1.8415, 8.2042, 1.355852, new Rotation3d(0, 0, 4.71238898)), // 6
            new Pose3d(-.0381, 5.547868, 1.451102, new Rotation3d(0, 0, 0)), // 7
            new Pose3d(-.0381, 4.982718, 1.451102, new Rotation3d(0, 0, 0)), // 8
            new Pose3d(0.356108, .883666, 1.355852, new Rotation3d(0, 0, 1.047197551)), // 9
            new Pose3d(1.461516, .245872, 1.355852, new Rotation3d(0, 0, 1.047197551)), // 10
            new Pose3d(11.9047, 3.7132, 1.3208, new Rotation3d(0, 0, 5.235987756)), // 11
            new Pose3d(11.9047, 4.4983, 1.3208, new Rotation3d(0, 0, 1.047197551)), // 12
            new Pose3d(11.22019, 4.105148, 1.3208, new Rotation3d(0, 0, 3.141492654)), // 13
            new Pose3d(5.3207, 4.105148, 1.3208, new Rotation3d(0, 0, 0)), // 14
            new Pose3d(4.641, 4.49834, 1.3208, new Rotation3d(0, 0, 2.094395102)), // 15
            new Pose3d(4.641, 3.713226, 1.3208, new Rotation3d(0, 0, 4.188790205)), // 16
    };

    public void applyVisionUpdate(long tagNumber, Translation3d targetTranslation, double ts) {
        if(tagNumber<=0)
        {
            return;
        }
        Pose3d targetTagPos = AprilTagList[(((int) tagNumber) - 1)];
        targetTranslation.rotateBy(new
        Rotation3d(0,0,m_gyro.getRotation2d().getRadians()));

        Pose3d robotPos = targetTagPos.transformBy(
        new Transform3d(targetTranslation, new Rotation3d(0, 0,
        0)));

        this.posEstimate.addVisionMeasurement(new
        Pose2d(robotPos.getX(),robotPos.getY(),m_gyro.getRotation2d()), ts);
    }
}
