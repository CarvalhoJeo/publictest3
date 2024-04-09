// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.Java_Is_UnderControl.Control.PidConfig;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.Java_Is_UnderControl.Logging.Pose2dLogEntry;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.PhotonVision.PhotonVision;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.Java_Is_UnderControl.Util.Util;
import frc.Java_Is_UnderControl.Vision.Odometry.LimelightPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.MultiCameraPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.PhotonVisionPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandsConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Joysticks.ControlBoard;

public class SwerveSubsystem extends OdometryEnabledSwerveSubsystem implements ISwerve {

  private ControlBoard controller = ControlBoard.getInstance();

  public boolean twoApriltagsDetected = false;

  public boolean alignedWithAngle;

  private boolean notAbleToRotate = false;

  private boolean overrideRotationAuto = false;

  private final int MAX_LOST_SPEAKER_FRAMES = 2;

  private final int MAX_LOST_NOTE_FRAMES = 2;

  private int lostNotesFrameCounter = this.MAX_LOST_NOTE_FRAMES;

  private double lastTxMeasurement;

  private static SwerveSubsystem swerveInstance = null;

  private StabilizeChecker alignedToSpeakerStabilizeChecker = new StabilizeChecker();

  private Rotation2d angleAlign = new Rotation2d(Units.degreesToRadians(0));

  PIDController pidTrackNote = new PIDController(CommandsConstants.P_TrackNote, CommandsConstants.I_TrackNote,
      CommandsConstants.D_TrackNote);

  PIDController pidTrackNoteAuto = new PIDController(CommandsConstants.P_TrackNote_Auto,
      CommandsConstants.I_TrackNote_Auto,
      CommandsConstants.D_TrackNote_Auto);

  Pose2d poseVision2D = new Pose2d();

  Pose3d poseVision3D = new Pose3d();

  LimelightHelpers.PoseEstimate limelightMeasurement;

  private Pose2dLogEntry poseVisionLogEntry = new Pose2dLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/PoseVision");

  private CustomBooleanLogger isAlignedWithSpeakerLogEntry = new CustomBooleanLogger(
      "/SwerveSubsystem/SpeakerAlign/isAligned");

  private CustomBooleanLogger isSwerveInAutoShootZoneLogEntry = new CustomBooleanLogger(
      "/SwerveSubsystem/SpeakerAlign/isInAutoShootZone");

  private CustomBooleanLogger isSwerveInAutoShootVelocityLogEntry = new CustomBooleanLogger(
      "/SwerveSubsystem/SpeakerAlign/isInAutoShootVelocity");

  private CustomBooleanLogger isSwerveInShootVelocityLogEntry = new CustomBooleanLogger(
      "/SwerveSubsystem/SpeakerAlign/isInShootVelocity");

  private CustomDoubleLogger distanceToNearestAmpLogEntry = new CustomDoubleLogger(
      "/SwerveSubsystem/AutoAmp/DistanceToNearestAmp");

  private CustomDoubleLogger distanceToSpeakerAllianceLogEntry = new CustomDoubleLogger(
      "/SwerveSubsystem/DistanceToSpeakerAlliance");

  private CustomBooleanLogger isAtAmpPositionLogEntry = new CustomBooleanLogger(
      "/SwerveSubsystem/AutoAmp/isAtAmpPosition");

  private String state = "DRIVE_ALIGN_ANGLE_BUTTON";

  private CustomStringLogger stateLogEntry = new CustomStringLogger("/SwerveSubsystem/State");

  private double targetNotePositionX;

  private PIDController xVelToAprilTag = new PIDController(0.01, 0, 0);
  private PIDController yVelToAprilTag = new PIDController(0.01, 0, 0);

  private PhotonVision shooterTopCamera = new PhotonVision("Microsoft_LifeCam_HD-3000");

  private PhotonVision intakeNoteDetecionCamera = new PhotonVision("Microsoft_LifeCam_HD-3000 (1)");

  public static SwerveSubsystem getInstance(File directory) {
    Transform3d robotToCam = new Transform3d(new Translation3d(-0.185, -0.272, 0.27),
        new Rotation3d(0, Units.degreesToRadians(-37.5), Units.degreesToRadians(-70)));
    if (swerveInstance == null) {
      List<PoseEstimator> listOfEstimators = new ArrayList<>();
      List<PoseEstimator> listOfEstimatorsForAuto = new ArrayList<>();
      PoseEstimator limelight3G_Two_TagsOnly = new LimelightPoseEstimator("limelight-back", true, true);
      PoseEstimator limelight2_Two_TagsOnly = new LimelightPoseEstimator("limelight-left", true);
      PoseEstimator arducam_Two_TagsOnly = new PhotonVisionPoseEstimator(new PhotonCamera("Arducam"), robotToCam, true);
      PoseEstimator limelight3G = new LimelightPoseEstimator("limelight-back", false);
      PoseEstimator limelight2 = new LimelightPoseEstimator("limelight-left", false);
      PoseEstimator arducam = new PhotonVisionPoseEstimator(new PhotonCamera("Arducam"), robotToCam, true);
      listOfEstimatorsForAuto.add(limelight3G_Two_TagsOnly);
      listOfEstimatorsForAuto.add(limelight2_Two_TagsOnly);
      listOfEstimatorsForAuto.add(arducam_Two_TagsOnly);
      listOfEstimators.add(limelight3G);
      listOfEstimators.add(limelight2);
      listOfEstimators.add(arducam);
      PoseEstimator estimatorMultiCamera_Auto = new MultiCameraPoseEstimator(listOfEstimatorsForAuto);
      PoseEstimator estimatorMultiCamera = new MultiCameraPoseEstimator(listOfEstimators);
      swerveInstance = new SwerveSubsystem(directory, estimatorMultiCamera, limelight3G_Two_TagsOnly);
      limelight3G.setSwerveDriveForGyroReference(swerveInstance);
      limelight2.setSwerveDriveForGyroReference(swerveInstance);
      limelight2_Two_TagsOnly.setSwerveDriveForGyroReference(swerveInstance);
      limelight3G_Two_TagsOnly.setSwerveDriveForGyroReference(swerveInstance);
    }
    return swerveInstance;
  }

  public static SwerveSubsystem getInstance() {
    if (swerveInstance == null) {
      DriverStation.reportError("YAGSL Configuration not provided", true);
    }
    return swerveInstance;

  }

  private SwerveSubsystem(File directory, PoseEstimator poseEstimatorTeleOp, PoseEstimator poseEstimatorAuto) {
    super(directory, new OdometryEnabledSwerveConfig(
        SwerveConstants.MAX_VEL,
        SwerveConstants.MAX_ROTATION_VEL,
        Alliance.Red,
        new SwervePathPlannerConfig(
            AutoConstants.TranslationPID,
            AutoConstants.angleAutoPID,
            4.5),
        poseEstimatorAuto,
        poseEstimatorTeleOp,
        new PidConfig(0.7, 0, 0)));
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    this.updateLogs();
  }

  @Override
  public void periodic() {
    super.periodic();
    this.updateLogs();
  }

  @Override
  public void setModulesToBrake() {
    swerveDrive.setMotorIdleMode(true);
  }

  @Override
  public void setModulesToCoast() {
    swerveDrive.setMotorIdleMode(false);
  }

  @Override
  public void driveAlignAngleButton() {
    if (this.isControllerAskingForRotation()) {
      this.driveRotating();
      return;
    }
    if (controller.notUsingJoystick()) {
      ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
          controller.getXtranslation());
      this.driveFieldOrientedLockedAngle(desiredSpeeds, angleAlign);
      this.state = "DRIVE_ALIGN_ANGLE_BUTTON";
    } else {
      this.driveAlignAngleJoy();
    }
  }

  @Override
  public void driveAlignAngleJoy() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    double angleTarget = Math.atan2(controller.getCOS_Joystick(), controller.getSIN_Joystick());
    this.driveFieldOrientedLockedAngle(desiredSpeeds, Rotation2d.fromRadians(angleTarget));
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
  }

  public void driveRotating() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    ChassisSpeeds speedsRotate;
    if (this.controller.rotateLeft()) {
      speedsRotate = new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond,
          SwerveConstants.ROTATION_BUTTON_SPEED);
    } else if (this.controller.rotateRight()) {
      speedsRotate = new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond,
          -SwerveConstants.ROTATION_BUTTON_SPEED);
    } else {
      speedsRotate = desiredSpeeds;
    }
    this.driveFieldOriented(speedsRotate);
  }

  private boolean isControllerAskingForRotation() {
    return controller.rotateLeft() || controller.rotateRight();
  }

  @Override
  public void driveAlignSpeaker() {
    notAbleToRotate = true;
    if (this.controller.isForcingDriverControl()) {
      this.driveAlignAngleButton();
      return;
    }
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    if (alliance == Alliance.Red) {
      this.driveAimingAtPosition(desiredSpeeds, FieldConstants.redGoalPos, new Rotation2d(Math.PI));
    } else {
      this.driveAimingAtPosition(desiredSpeeds, FieldConstants.blueGoalPos);
    }
    this.state = "DRIVE_ALIGN_SPEAKER";
  }

  @Override
  public void driveAlignShootOverStage() {
    notAbleToRotate = true;
    if (this.controller.isForcingDriverControl()) {
      this.driveAlignAngleButton();
      return;
    }
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    if (alliance == Alliance.Red) {
      this.driveAimingAtPosition(desiredSpeeds, FieldConstants.redFeedPos, new Rotation2d(Math.PI));
    } else {
      this.driveAimingAtPosition(desiredSpeeds, FieldConstants.blueFeedPos);
    }
    this.state = "DRIVE_ALIGN_SHOOT_OVER_STAGE";
  }

  @Override
  public void driveAlignNote() {
    notAbleToRotate = true;
    if (this.controller.isForcingDriverControl()) {
      driveAlignAngleButton();
      return;
    }
    boolean isDetectingNote = intakeNoteDetecionCamera.hasTarget();
    if (isDetectingNote) {
      this.lostNotesFrameCounter = 0;
      this.targetNotePositionX = intakeNoteDetecionCamera.targetPositionX();
      ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
          controller.getXtranslation());
      driveOmegaPidFieldOriented(desiredSpeeds, 0, this.targetNotePositionX, this.pidTrackNote, false);
    } else {
      this.lostNotesFrameCounter++;
      if (this.lostNotesFrameCounter <= this.MAX_LOST_SPEAKER_FRAMES) {
        ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
            controller.getXtranslation());
        driveOmegaPidFieldOriented(desiredSpeeds, 0, this.lastTxMeasurement, this.pidTrackNote, false);
      } else {
        driveAlignAngleButton();
      }
    }
    this.state = "DRIVE_ALIGN_NOTE";
  }

  @Override
  public void driveToAmp() {
    notAbleToRotate = true;
    Pose2d targetPose;
    if (this.getPose().getX() > FieldConstants.midFieldLine) {
      targetPose = FieldConstants.redAmp;
    } else {
      targetPose = FieldConstants.blueAmp;
    }
    this.driveToPose(
        new Pose2d(targetPose.getX(), targetPose.getY(), this.angleAlign));
    this.state = "DRIVE_TO_AMP";
  }

  @Override
  public void alignWithAmpAprilTag() {
    double angleAlignAmp = defineAngleToAlignWithAmp();
    this.alignWithAprilTag(0.0, SwerveConstants.ALIGNED_WITH_AMP_GOAL, angleAlignAmp);
    this.state = "ALIGN_WITH_AMP_APRIL_TAG";
  }

  private double defineAngleToAlignWithAmp() {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      return 90;
    } else {
      return -90;
    }
  }

  private void alignWithAprilTag(double targetX, double targetY, double targetAngle) {
    if (this.controller.isForcingDriverControl()) {
      this.driveAlignAngleButton();
      return;
    }
    if (!this.shooterTopCamera.hasTarget()) {
      this.driveAlignAngleButton();
      return;
    }
    double aprilTagX = this.shooterTopCamera.targetPositionX();
    double aprilTagY = this.shooterTopCamera.targetPositionY();
    double velocityX = -xVelToAprilTag.calculate(aprilTagX, targetX);
    double velocityY = yVelToAprilTag.calculate(aprilTagY, targetY);
    ChassisSpeeds targetSpeeds = this.inputsToChassisSpeeds(velocityX, velocityY);
    this.driveRobotOrientedLockedAngle(targetSpeeds, Rotation2d.fromDegrees(targetAngle));
  }

  @Override
  public double getDistanceToNearestAmp() {
    if (this.getPose().getX() > FieldConstants.midFieldLine) {
      return this.getDistanceToPosition(FieldConstants.redAmp.getTranslation());
    } else {
      return this.getDistanceToPosition(FieldConstants.blueAmp.getTranslation());
    }
  }

  @Override
  public boolean isSwerveAlignedWithSpeaker() {
    boolean isAlignedWithSpeaker = this.alignedToSpeakerStabilizeChecker.isStableInCondition(
        () -> this.isAtTargetHeading(SwerveConstants.DEADBAND_ALIGNED_WITH_SPEAKER));
    return isAlignedWithSpeaker;
  }

  @Override
  public boolean isAtAmpAngle() {
    return this.isAtTargetHeading(5);
  }

  @Override
  public void setAngleToAmp() {
    double angleAlignAmp = defineAngleToAlignWithAmp();
    this.angleAlign = new Rotation2d(Units.degreesToRadians(angleAlignAmp));
  }

  @Override
  public boolean isAtAmpPosition() {
    return this.isAtTargetPose(SwerveConstants.DEADBAND_AT_AMP_POSITION_X, SwerveConstants.DEADBAND_AT_AMP_POSITION_Y,
        5);
  }

  @Override
  public boolean isAlignedWithAmp() {
    return this.isAlignedWithAprilTag(
        0,
        SwerveConstants.ALIGNED_WITH_AMP_GOAL,
        SwerveConstants.DEADBAND_ALIGNED_WITH_AMP_X,
        SwerveConstants.DEADBAND_ALIGNED_WITH_AMP_Y);
  }

  private boolean isAlignedWithAprilTag(double targetX, double targetY, double deadbandX, double deadbandY) {
    if (!this.shooterTopCamera.hasTarget()) {
      return false;
    }
    double aprilTagX = this.shooterTopCamera.targetPositionX();
    double aprilTagY = this.shooterTopCamera.targetPositionY();
    boolean isAlignedInX = Util.inRange(aprilTagX, targetX - deadbandX, targetX + deadbandX);
    boolean isAlignedInY = aprilTagY > -6;
    double headingDegrees = getHeading().getDegrees();
    boolean alignedWithAngle = Util.inRange(Math.abs(headingDegrees), Math.abs(angleAlign.getDegrees()) - 5,
        Math.abs(angleAlign.getDegrees()) + 5);
    return isAlignedInX && isAlignedInY && alignedWithAngle;
  }

  @Override
  public boolean isSwerveInAutoShootZone() {
    boolean isInFrontOfSpeaker = this.getPose().getY() > 2.9;
    boolean isNearSpeaker = distanceFromSpeaker() < 4.5;
    return isNearSpeaker && isInFrontOfSpeaker;
  }

  @Override
  public boolean isSwerveInVelocityToAutoShoot() {
    double velocity = this.getAbsoluteRobotVelocity();
    return velocity < 1;
  }

  @Override
  public boolean isSwerveInVelocityToShoot() {
    double velocity = this.getAbsoluteRobotVelocity();
    return velocity < 2.5;
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    if (overrideRotationAuto && intakeNoteDetecionCamera.hasTarget()) {
      return Optional.of(swerveDrive.getOdometryHeading()
          .minus(new Rotation2d(Units.degreesToRadians(intakeNoteDetecionCamera.targetPositionX()))));
    } else {
      return Optional.empty();
    }
  }

  @Override
  public void overrideRotationAuto(boolean Override) {
    overrideRotationAuto = Override;
  }

  private void driveOmegaPidFieldOriented(ChassisSpeeds speeds, double setpoint, double measurement, PIDController pid,
      boolean invertOmega) {
    ChassisSpeeds speedsPID = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        invertOmega ? -pid.calculate(measurement, setpoint) : pid.calculate(measurement, setpoint));
    this.driveFieldOriented(speedsPID);
  }

  public double distanceFromSpeaker() {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      return getPose().getTranslation().getDistance(FieldConstants.redGoalPos);
    } else {
      return getPose().getTranslation().getDistance(FieldConstants.blueGoalPos);
    }
  }

  public double distanceFromFeedPos() {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      return getPose().getTranslation().getDistance(FieldConstants.redFeedPos);
    } else {
      return getPose().getTranslation().getDistance(FieldConstants.blueFeedPos);
    }
  }

  /*
   * ANGLE CONTROL
   */

  public void setAngle90() {
    this.angleAlign = new Rotation2d(Units.degreesToRadians(90));
  }

  public void setAngle180() {
    this.angleAlign = new Rotation2d(Units.degreesToRadians(180));
  }

  public void setAngleNeg90() {
    this.angleAlign = new Rotation2d(Units.degreesToRadians(-90));
  }

  public void setAngle0() {
    this.angleAlign = new Rotation2d(Units.degreesToRadians(0));
  }

  public void setAngle45() {
    this.angleAlign = new Rotation2d(Units.degreesToRadians(35));
  }

  public void setAngleNeg45() {
    this.angleAlign = new Rotation2d(Units.degreesToRadians(-45));
  }

  protected void updateLogs() {
    this.poseVisionLogEntry.appendRadians(this.poseVision2D);
    this.isAlignedWithSpeakerLogEntry.append(isSwerveAlignedWithSpeaker());
    this.distanceToSpeakerAllianceLogEntry.append(distanceFromSpeaker());
    this.isSwerveInAutoShootZoneLogEntry.append(isSwerveInAutoShootZone());
    this.isSwerveInShootVelocityLogEntry.append(isSwerveInVelocityToShoot());
    this.isSwerveInAutoShootVelocityLogEntry.append(isSwerveInVelocityToAutoShoot());
    this.distanceToNearestAmpLogEntry.append(getDistanceToNearestAmp());
    this.isAtAmpPositionLogEntry.append(isAtAmpPosition());
    this.stateLogEntry.append(this.state);
  }

}
