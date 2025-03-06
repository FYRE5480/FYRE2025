package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.CameraWebsocketClient;
import frc.robot.util.CameraWebsocketClient.Apriltag;
import frc.robot.util.CameraWebsocketClient.Info;
import edu.wpi.first.wpilibj.Timer;


public class Vision {
    public static enum Side {FRONT, LEFT, BACK, RIGHT}

    private String ip;
    private ArrayList<CameraWebsocketClient> camClientList = new ArrayList<CameraWebsocketClient>();
    private HashMap<String, Integer[]> apriltagPoses; // Hashmap of [angle, x, y] where angle is the angle of the tag in rads, where 
    private Timer timer = new Timer();

    private PIDController turnPID = new PIDController(8.192, 0.0, 0.0008);
    private PIDController movePID = new PIDController(10.05, 0.0, 0.0);
    private ChassisSpeeds prevChassisSpeeds;
    private double prevTime;

    public static class CameraPair{
        public int cam1;
        public int cam2;

        public double cam1Angle;
        public double cam2Angle;

        public double cam1XOffset;
        public double cam1YOffset;

        public double cam2XOffset;
        public double cam2YOffset;
        public CameraPair(int cam1, int cam2, double cam1Angle, double cam2Angle, double cam1XOffset, double cam1YOffset, double cam2XOffset, double cam2YOffset) {
            this.cam1 = cam1;
            this.cam2 = cam2;
            this.cam1Angle = cam1Angle;
            this.cam2Angle = cam2Angle;
            this.cam1XOffset = cam1XOffset;
            this.cam1YOffset = cam1YOffset;
            this.cam2XOffset = cam2XOffset;
            this.cam2YOffset = cam2YOffset;
        }
    }
    public static class RobotPositionOffset{
        public double xOffset;
        public double yOffset;
        public double angleOffset;

        public RobotPositionOffset(double xOffset, double yOffset, double angleOffset) {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.angleOffset = angleOffset;
        }
    }
    
    public Vision(String ipAddress, int[] cameraRotation, HashMap<String, Integer[]> apriltagPoses) {
        // This constructor is not ideal but it works for the example. IRL you would want to use the other constructor so you can still have a list of cameras outside of the Interface.
        // Maybe I will make this the only class that you need to use with the cameras then it will be fine.
        // Camera Rotation is the rotation of each camera in degrees. 0 is the default rotation.
        // Apriltag Angles is a hashmap of the apriltag id to the angle of the tag in degrees. 0 is facing the camera.

        this.ip = ipAddress;
        this.apriltagPoses = apriltagPoses;
        boolean failed = false;
        int i = 0;
        while(!failed) {
            System.out.println("Trying to commect to: " + ip + ":" + (i + 50000));
            CameraWebsocketClient newCam = new CameraWebsocketClient(ip + ":" + (i + 50000), 1000);
            newCam.setupConnection();

            if(newCam.isConnected()) {
                newCam.setRotation(cameraRotation[i]);
                camClientList.add(newCam);
                i++;
            } else {
                failed = true;
            }
        }

        turnPID.enableContinuousInput(-180, 180);
        turnPID.setSetpoint(0);
        movePID.enableContinuousInput(-180, 180);
        movePID.setSetpoint(0);
    }

    public Vision(CameraWebsocketClient[] camList, HashMap<String, Integer[]> apriltagPoses) {
        this.apriltagPoses = apriltagPoses;
        for(CameraWebsocketClient newCam : camList) {
            if(newCam.isConnected()) {
                camClientList.add(newCam);
            }
        }

        // Erm, what the sigma? IS this corect? I think it is but I am not sure
        turnPID.enableContinuousInput(-180, 180);
        turnPID.setSetpoint(0);
        movePID.enableContinuousInput(-180, 180);
        movePID.setSetpoint(0);
    }

    public void clear(){
        for(CameraWebsocketClient cam : camClientList) {
            cam.clear();
        }
    }

    public ChassisSpeeds frontToSide(ChassisSpeeds inSpeeds, Side side){
        switch(side) {
            case FRONT:
                return inSpeeds;
            case LEFT:
                return new ChassisSpeeds(-inSpeeds.vyMetersPerSecond, inSpeeds.vxMetersPerSecond, inSpeeds.omegaRadiansPerSecond);
            case RIGHT:
                return new ChassisSpeeds(inSpeeds.vyMetersPerSecond, -inSpeeds.vxMetersPerSecond, inSpeeds.omegaRadiansPerSecond);
            case BACK:
                return new ChassisSpeeds(-inSpeeds.vxMetersPerSecond, -inSpeeds.vyMetersPerSecond, inSpeeds.omegaRadiansPerSecond);
            default:
                return inSpeeds;
          }
          
    }

    public double getZAngle(int maxTags) {
        // This function returns the average calculated angle of the robot in degrees on the z axis, aka the only one the robot turns on. Limit the number of tags to use with maxTags if you want.

        double ZAngle = 0;
        double numTags = 0;
        for(CameraWebsocketClient cam : camClientList) {
            List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();

            System.out.println(tags.size());

            for (CameraWebsocketClient.Apriltag tag : tags) {
                Integer[] tagAngleArray = apriltagPoses.getOrDefault(tag.tagId, new Integer[]{0, 0, 0});
                int tagAngle = tagAngleArray[0];
                ZAngle += tagAngle * (180/Math.PI) + cam.getRotation() + tag.orientation[1];
                numTags++;
                if(numTags == maxTags) {
                    break;
                }
            }
            if(numTags == maxTags) {
                break;
            }
        }
        if(numTags == 0) {
            return 69420.0; // nice
        }
        return ZAngle/numTags % 360;
    }

    public double getZAngle() {
        // W overload
        return getZAngle(4);
    }

    /**
     * Returns an INCOMPLETE ChassisSpeeds object - only the rotation unit - for locking on to a tag 

     * @param camIndex - the index of the desired camera to use
     * @param tagId - the apriltag ID to search for, null if no preference
     * @return speeds - the ChassisSpeeds object for the rotation to take
     */
    public ChassisSpeeds lockonTagSpeeds(int camIndex, String tagId) {
        Apriltag tag;
        if (tagId != null) tag = decideTag(camIndex, tagId);
        else tag = decideTag(camIndex);
        if (tag == null) return null;

        return new ChassisSpeeds(
            0,
            0,
            turnPID.calculate(tag.horizontalAngle)
        );
    }

    /**
     * This function returns the relative position of the tag to the camera in the camera's frame of reference.

     * @param camIndex - the index of the desired camera to use
     * @param tagId - the apriltag ID to search for, null if no preference
     * @return speeds - the ChassisSpeeds object for the robot to take
     */
    public ChassisSpeeds getTagDrive(int camIndex, String[] tagIds, Side side, double cameraHorizontalAngle, double xOffset, double yOffset, PIDController turnPIDArg) {
        // The position is returned as a 3 element array of doubles in the form [x, y, z]
        // The position is in meters.
        double turnSpeed;
        Apriltag tag;
        if (tagIds != null) tag = decideTag(camIndex, tagIds);
        else tag = decideTag(camIndex);        
        if(tag == null)return null;


        double e; 
        if (yOffset == 0) e = -Math.PI/2;
        else e = Math.atan(-xOffset/(yOffset)) + (Math.PI * (Math.signum(yOffset)-1)/2);
        double f = Math.sqrt(xOffset * xOffset + yOffset * yOffset);

        double xDist = tag.distance * Math.sin(tag.horizontalAngle) + Math.cos(cameraHorizontalAngle + e);
        double yDist = tag.distance * Math.cos(tag.horizontalAngle) + Math.sin(cameraHorizontalAngle + e);

        double totDist = Math.sqrt(xDist * xDist + yDist * yDist);


        // if (turnPIDArg != null) turnSpeed = turnPIDArg.calculate(tag.orientation[0] + cameraHorizontalAngle); // This seems to be fine it may need to be negative but idk
        // else 
        turnSpeed = turnPID.calculate(tag.orientation[1] - cameraHorizontalAngle);   
        double moveSpeed = movePID.calculate(totDist); // I do not know if this is correct - it makes some sense but idk

        // Look at this! Max is doing a weird normalization thing again!
        // double xMove = ((tag.position[2] - xOffset) / (Math.abs(tag.position[0]) + Math.abs(tag.position[2]))) * moveSpeed;
        // double yMove = ((tag.position[0] - yOffset) / (Math.abs(tag.position[0]) + Math.abs(tag.position[2]))) * moveSpeed;

        double xMove = (xDist / totDist) * moveSpeed;
        double yMove = (yDist / totDist) * moveSpeed;


        return frontToSide(new ChassisSpeeds(
            -DriveConstants.highDriveSpeed * xMove,
            DriveConstants.highDriveSpeed * yMove,
            turnSpeed), side);
    }

    public ChassisSpeeds getTagDrive(int camIndex) {
        return getTagDrive(camIndex, null, Side.BACK, 0, 0, 0, null);
    }

    public ChassisSpeeds getTagDrive(CameraPair cams, String[] tagIds, Side side, RobotPositionOffset offsets, PIDController turnPidArg) {
        // The position is returned as a 3 element array of doubles in the form [x, y, z]
        // The position is in meters.

        double angleOffset = offsets.angleOffset;
        double xOffset = offsets.xOffset;
        double yOffset = offsets.yOffset;

        ChassisSpeeds cam1Speed = getTagDrive(cams.cam1, tagIds, side, cams.cam1Angle + angleOffset, cams.cam1XOffset + xOffset, cams.cam1YOffset + yOffset, turnPidArg);
        ChassisSpeeds cam2Speed = getTagDrive(cams.cam2, tagIds, side, cams.cam2Angle + angleOffset, cams.cam2XOffset + xOffset, cams.cam2YOffset + yOffset, turnPidArg);
        if (cam1Speed == null){
            return cam2Speed;
        }
        if (cam2Speed == null){
            return cam1Speed;
        }
        
        return new ChassisSpeeds(
            (cam1Speed.vxMetersPerSecond + cam2Speed.vxMetersPerSecond) / 2,
            (cam1Speed.vyMetersPerSecond + cam2Speed.vyMetersPerSecond) / 2,
            (cam1Speed.omegaRadiansPerSecond + cam2Speed.omegaRadiansPerSecond) / 2
        );
    }

    public ChassisSpeeds getPieceDrive(int camIndex, double cameraOffsetAngle, double xOffset, double yOffset) {
        CameraWebsocketClient cam = camClientList.get(camIndex);
        CameraWebsocketClient.Piece piece = cam.getPiece();
        
        if(piece == null) {
            return null;
        }
        
        double driveAngleModifier;
        if (piece.angle > VisionConstants.maxIntakeAngle) {
            driveAngleModifier = VisionConstants.misallignedPieceOffset / piece.distance; 
            // This math works in my head. Make the angle larger if the piece is closer, and less when it is farther. 
            //Asymptotic to 0, so it will always theoretically adjust.
        } else if (piece.angle < -VisionConstants.maxIntakeAngle) {
            driveAngleModifier = -VisionConstants.misallignedPieceOffset / piece.distance;
        } else {
            driveAngleModifier = 0;
        }

        double x = Math.cos(piece.angle + cam.getRotation() + driveAngleModifier) * piece.distance - xOffset;
        double y = Math.sin(piece.angle + cam.getRotation() + driveAngleModifier) * piece.distance - yOffset;
        

        double turnSpeed = turnPID.calculate(piece.angle-cameraOffsetAngle);
        double moveSpeed = movePID.calculate(piece.distance);

        double xMove = (x / Math.sqrt(x*x + y*y)) * moveSpeed;
        double yMove = (y / Math.sqrt(x*x + y*y)) * moveSpeed;

        return new ChassisSpeeds(
            DriveConstants.highDriveSpeed * xMove,
            DriveConstants.highDriveSpeed * yMove,
            turnSpeed
        );
    }

    public ChassisSpeeds getPieceDrive() {
        return getPieceDrive(VisionConstants.pieceDetectionCamIndex, 0, 0, 0);
    }

    private Apriltag decideTag(int camIndex) {
        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        Apriltag tag = null;

        if (tags.size() > 0) {
            Apriltag bestTag = tags.get(0);
            for (Apriltag t : tags) { // This is a weird way to do this but it works - I need to make this more efficient
                if(t.distance < bestTag.distance) {
                    tag = t;
                    break;
                }
            }
        }

        return tag;
    }

    private Apriltag decideTag(int camIndex, String tagId) {
        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        Apriltag tag = null;
        for (Apriltag t : tags) { // This is a weird way to do this but it works - I need to make this more efficient
            if(t.tagId.equals(tagId)) {
                tag = t;
                break;
            }
        }

        return tag;
    }

    private Apriltag decideTag(int camIndex, String tagIds[]) {
        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        List<String> tagIdList = Arrays.asList(tagIds);
        if (tags.size() == 0) {
            return null;
        }
        Apriltag bestTag = tags.get(0);
        for (Apriltag t : tags) { // This is a weird way to do this but it works - I need to make this more efficient
            if(tagIdList.contains(t.tagId) && t.distance <= bestTag.distance) {
                bestTag = t;
            }
        }

        return bestTag;
    }

    public Info getInfo() {
        return camClientList.get(0).getInfo();
    }
    @Override
    protected void finalize() throws Throwable {
        try {
            for (CameraWebsocketClient cam : camClientList) {
                if (cam.isConnected()) {
                    cam.disconnect();
                }
            }
        } finally {
            super.finalize();
        }
    }
}