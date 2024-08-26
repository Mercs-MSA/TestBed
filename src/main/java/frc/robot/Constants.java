package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class SATConstants {
        public static final class SUB{
            public static final double pivot = 47;
        }

        public static final class AMP{
            public static final double pivot = 95;
        }

        public static final class PODIUM{
            public static final double pivot = 42;
        }
        
        public static final class WING{
            public static final double pivot = 25;
        }

        public static final class SHUNT{
            public static final double pivot = 30;
        }

        public static final class NEW_SHUNT{
            public static final double pivot = 80;
        }

        public static final class START{
            public static final double pivot = 0.2;
        }
    }

    public static class ArmConstants{
        public static int leaderID = 20;
        public static int followerTalon = 7;
        public static double armEncoderOffsetRads = 0.0;
        public static boolean leaderInverted = true;

        /* Leader Motor PID Values */
        public static final double leaderKP = 1.5;
        public static final double leaderKI = 0.0;
        public static final double leaderKD = 0.0;
        public static final double leaderKF = 0.0;

        public static int rotorToSensorRatio = 1;
        public static int sensorToMechanismRatio = 1;
        public static Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static double tol = 0.6;
    }

    public static class ScoringConstants {
        public static ScoringMode currentScoringMode = ScoringMode.PODIUM;
        public enum ScoringMode {
            INTAKE,
            WING,
            AMP,
            SUBWOOFER,
            PODIUM,
            TRAP,
            AUTOAIM,
            NEW_SHUNT,
            START
        }
    }

    public static boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
        return (Math.abs(targetPose - currentPose) <= tolerance);
    }

    public static boolean isPoseWithinTol(Pose2d targetPose, Pose2d currentPose, Pose2d tol) {
        return Math.abs(targetPose.getTranslation().getX() - currentPose.getTranslation().getX()) <= tol.getTranslation().getX() &&
               Math.abs(targetPose.getTranslation().getY() - currentPose.getTranslation().getY()) <= tol.getTranslation().getY() &&
               Math.abs(targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees()) <= tol.getRotation().getDegrees();
    }   
}