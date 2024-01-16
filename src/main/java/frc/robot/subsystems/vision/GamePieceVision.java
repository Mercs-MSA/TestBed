package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GamePieceVision {
    private String cameraName;
    private PhotonCamera camera;
    private PhotonPipelineResult gamePieceResult = new PhotonPipelineResult();
    private boolean gamePieceHasTargets = false;
    private List<PhotonTrackedTarget> gamePieceTargets;
    private PhotonTrackedTarget gamePieceBestTarget;
    double gamePieceYaw, gamePiecePitch, gamePieceSkew, gamePieceAreaPercent = 999.0;

    public GamePieceVision(String cameraName){
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
    }

    public void periodic(){
        gamePieceResult = camera.getLatestResult();
        
        gamePieceHasTargets = gamePieceResult.hasTargets();

        if (gamePieceHasTargets){
            gamePieceTargets = gamePieceResult.getTargets();
            gamePieceBestTarget = gamePieceResult.getBestTarget();

            gamePieceYaw = gamePieceBestTarget.getYaw();
            // gamePiecePitch = gamePieceBestTarget.getPitch();
            // gamePieceSkew = gamePieceBestTarget.getSkew();
            // gamePieceAreaPercent = gamePieceBestTarget.getArea();
        }
        else{
            gamePieceYaw = 999.0;
            // gamePiecePitch = 999.0;
            // gamePieceSkew = 999.0;
            // gamePieceAreaPercent = 999.0;
        }

        // Update SmartDashboard for game piece
        SmartDashboard.putNumber("Game Piece Yaw", gamePieceYaw);
        // SmartDashboard.putNumber("Game Piece Pitch", gamePiecePitch);
        // SmartDashboard.putNumber("Game Piece Skew", gamePieceSkew);
        // SmartDashboard.putNumber("Game Piece Area Percent", gamePieceAreaPercent);
    }

    /**
     * Gets the Yaw angle of the game piece.
     * @return The Yaw angle.
     */
    public double getGamePieceYaw(){
        return gamePieceYaw;
    }

    /**
     * Gets the Pitch angle of the game piece.
     * @return The Pitch angle.
     */
    public double getGamePiecePitch(){
        return gamePiecePitch;
    }

    /**
     * Gets the Skew angle of the game piece.
     * @return The Skew angle.
     */
    public double getGamePieceSkew(){
        return gamePieceSkew;
    }

    /**
     * Gets the Area Percent of the game piece.
     * @return The Area Percent.
     */
    public double getGamePieceAreaPercent(){
        return gamePieceAreaPercent;
    }

}
