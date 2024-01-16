package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ApriltagVision {

    private String cameraName;
    private PhotonCamera camera;
    private PhotonPipelineResult aprilTagResult;
    private boolean aprilTagHasTargets;
    private List<PhotonTrackedTarget> aprilTagTargets;
    private PhotonTrackedTarget aprilTagBestTarget;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator poseEstimator;
    private int fiducialID;
    private Transform3d robotToCam;
    private double aprilTagX, aprilTagY, aprilTagZAngle, aprilTagZ = -1;
    private Pose3d globalPoseEstimate;

    public ApriltagVision(String cameraName, Transform3d robotToCam) throws IOException{
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        aprilTagResult = new PhotonPipelineResult();
        aprilTagHasTargets = false;
        aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getName() + "/2024-crescendo.json");
        this.robotToCam = robotToCam; 
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
    }

    public void periodic(){
        aprilTagResult = camera.getLatestResult();
        
        aprilTagHasTargets = aprilTagResult.hasTargets();

        if (aprilTagHasTargets) {
            aprilTagTargets = aprilTagResult.getTargets();
            aprilTagBestTarget = aprilTagResult.getBestTarget();

            fiducialID = aprilTagBestTarget.getFiducialId();
            aprilTagX = aprilTagBestTarget.getBestCameraToTarget().getX();
            aprilTagY = aprilTagBestTarget.getBestCameraToTarget().getY();
            aprilTagZ = aprilTagBestTarget.getBestCameraToTarget().getZ();
            aprilTagZAngle = aprilTagBestTarget.getBestCameraToTarget().getRotation().getAngle();
        } 
        else {
            fiducialID = -1;
            aprilTagX = -1.0;
            aprilTagY = -1.0;
            aprilTagZ = -1.0;
            aprilTagZAngle = -1.0;
        }

        // Update SmartDashboard for AprilTag
        SmartDashboard.putNumber(cameraName + " Fiducial ID", fiducialID);
        SmartDashboard.putNumber(cameraName + " AprilTag X (m)", aprilTagX);
        SmartDashboard.putNumber(cameraName + " AprilTag Y (m)", aprilTagY);
        SmartDashboard.putNumber(cameraName + " AprilTag Z (m)", aprilTagZ);
        SmartDashboard.putNumber(cameraName + " AprilTag Z Angle", aprilTagZAngle);

    }

    public void updateEstimatedGlobalPose() {
        poseEstimator.update();
        globalPoseEstimate = poseEstimator.update().get().estimatedPose;
    }

    public Pose3d getGlobalPoseEstimate() {
        return this.globalPoseEstimate;
    }

    /**
     * Gets the Fiducial ID of the AprilTag.
     * @return The Fiducial ID.
     */
    public int getFiducialID(){
        return fiducialID;
    }

    /**
     * Gets the X coordinate of the AprilTag in meters.
     * @return The X coordinate.
     */
    public double getAprilTagX(){
        return aprilTagX;
    }

    /**
     * Gets the Y coordinate of the AprilTag in meters.
     * @return The Y coordinate.
     */
    public double getAprilTagY(){
        return aprilTagY;
    }

    /**
     * Gets the Z coordinate of the AprilTag in meters.
     * @return The Z coordinate.
     */
    public double getAprilTagZ(){
        return aprilTagZ;
    }

    /**
     * Gets the Z angle of the AprilTag in degrees.
     * @return The Z angle.
     */
    public double getAprilTagZAngle(){
        return aprilTagZAngle * (180 / Math.PI);
    }
}
