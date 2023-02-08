// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonvisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonvisionSubsystem. */
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;
  private final Field2d m_field = new Field2d();

  public PhotonvisionSubsystem() {
    // Set up a test arena of two apriltags at the center of each driver station set
    final AprilTag tag18 =
            new AprilTag(
                    18,
                    new Pose3d(
                            new Pose2d(
                                    FieldConstants.length,
                                    FieldConstants.width / 2.0,
                                    Rotation2d.fromDegrees(180))));
    final AprilTag tag01 =
            new AprilTag(
                    01,
                    new Pose3d(new Pose2d(0.0, FieldConstants.width / 2.0, Rotation2d.fromDegrees(0.0))));
    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(tag18);
    atList.add(tag01);

    // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
    AprilTagFieldLayout atfl =
            new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

    // Forward Camera
    photonCamera =
            new PhotonCamera(
                    VisionConstants
                            .cameraName); // Change the name of your camera here to whatever it is in the
    // PhotonVision UI.

    // Create pose estimator
    photonPoseEstimator =
            new PhotonPoseEstimator(
                    atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.robotToCam);
    
    SmartDashboard.putData("PhotonVision", m_field);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    var result = photonCamera.getLatestResult();
    if(result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
