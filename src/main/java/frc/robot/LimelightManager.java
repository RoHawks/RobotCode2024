package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightManager 
{
    private LimelightInformation[] mLimeLights;
    private CameraBlendMethod mCameraBlendMethod = CameraBlendMethod.Prioritize;

    
    public static final int WEST_CAMERA = 0;
    public static final int EAST_CAMERA = 1;


    public static final int PIPELINE_FOR_DRIVE_BY_SHOOTING = 0;
    public static final int PIPELINE_FOR_AUTO_AIM = 1;

    public void SetPipeline(int pPipeline)
    {
        for(int i =0; i < mLimeLights.length; i++)
        {
            mLimeLights[i].SetPipeline(pPipeline);
        }
    }

    private String GetNameForCamera(int pCamera)
    {
        return pCamera == WEST_CAMERA ? "West" : "East";
    }

    private static LimelightManager mInstance  = new LimelightManager();

    public static LimelightManager GetInstance()
    {
        return mInstance;
    }




    private LimelightManager()
    {
        mLimeLights = new LimelightInformation[2];
        mLimeLights[WEST_CAMERA] = new LimelightInformation( "west");
        mLimeLights[EAST_CAMERA] = new LimelightInformation( "east");
    }

    public void calculateBotpose()
    {
        for(int i = 0; i < mLimeLights.length; i++)
        {
            mLimeLights[i].calculateBotpose(
                i == WEST_CAMERA ? 0: 0//-0.532 : 0                //this is 22.5 incvhes plus a tiny fuidge factor from experimetnation
            );
        }
    }


    public void calculateCameraPoseTargetSpace()
    {
        for(int i = 0; i < mLimeLights.length; i++)
        {
            mLimeLights[i].calculateCameraPoseTargetSpace(
                i == WEST_CAMERA ? 0: 0//-0.532 : 0                //this is 22.5 incvhes plus a tiny fuidge factor from experimetnation
            );
        }
        //Now offset the west camera to be in the same "spot" as the east camera
        
    }

    private enum CameraBlendMethod
    {
        Prioritize,
        Average
    }

    public double[] getCameraPoseTargetSpaceForSpecificCamera(int pCamera)
    {
        return mLimeLights[pCamera].getCameraPoseTargetSpace();
    }

    public double[] getBotposeForSpecificCamera(int pCamera)
    {
        return mLimeLights[pCamera].getBotPose();
    }

    public double[] getBotPoseFromCameraBasedOnChassisSpeeds(ChassisSpeeds pSpeeds)
    {
        //if you are approaching from the robot's west side, prioritize the west camera, and vice versa
        //to do, implement me for real
        double leftRightSpeed = pSpeeds == null ? 0 : pSpeeds .vyMetersPerSecond;
        //SmartDashboard.putNumber("getBotPoseFromCameraBasedOnChassisSpeeds_pSpeeds_Y", leftRightSpeed);
        boolean isHeadedTowardsRobotsWest = leftRightSpeed > 0.1; //0.1 is arbitray here - just don't want to pick up "barely moving"
        if(isHeadedTowardsRobotsWest)
        {
            return getBotPoseByPriorityCamera(WEST_CAMERA);
        }
        else
        {
            return getBotPoseByPriorityCamera(EAST_CAMERA);
        }
    }

   

    public double[] getBotPoseByPriorityCamera(int pCamera)
    {
        double[] priorityCameraResults = getBotposeForSpecificCamera(pCamera);
        if(LimelightInformation.isValidBotPoseResults(priorityCameraResults))
        {
            SmartDashboard.putString("Using Camera As:" , GetNameForCamera(pCamera));
            return priorityCameraResults;
        }
        else
        {
            int otherCamera = (pCamera == EAST_CAMERA ? WEST_CAMERA : EAST_CAMERA);
            SmartDashboard.putString("Using Camera As:" , GetNameForCamera(otherCamera));
            return getBotposeForSpecificCamera(otherCamera);
        }
    }

    public double[] getCameraPoseTargetSpace()
    {
        if(mCameraBlendMethod == CameraBlendMethod.Prioritize)
        {
            if(mLimeLights[EAST_CAMERA].getCameraPoseTargetSpace() != null)
            {
                return mLimeLights[EAST_CAMERA].getCameraPoseTargetSpace();
            }
            else
            {
                return mLimeLights[WEST_CAMERA].getCameraPoseTargetSpace();
            }
        }
        else //average
        {
            double[] cameraPoseWest = mLimeLights[WEST_CAMERA].getCameraPoseTargetSpace();
            double[] cameraPoseEast = mLimeLights[EAST_CAMERA].getCameraPoseTargetSpace();

            if(cameraPoseWest != null && cameraPoseEast != null)
            {
                double[] returnValue = new double[cameraPoseEast.length];
                for(int i = 0; i < cameraPoseEast.length; i++)
                {
                    returnValue[i] = (cameraPoseWest[i] + cameraPoseEast[i]) /2.0;
                }
                return returnValue;
            }
            else
            {
                if(cameraPoseWest != null)
                {
                    return cameraPoseWest;
                }
                else
                {
                    return cameraPoseEast;
                }
            }

        }
    }


    
  private double lastRecordedX = 0; 
  private void logLimelightInfo(String pCameraName, double[] camerapose_targetspace)
  {

    
   if (camerapose_targetspace != null)
    {
      SmartDashboard.putNumber(pCameraName + "Lime: X change since last TS", camerapose_targetspace[0] - lastRecordedX);
      lastRecordedX = camerapose_targetspace[0];
      SmartDashboard.putNumber(pCameraName + "X displacement", camerapose_targetspace[0]);
      SmartDashboard.putNumber(pCameraName + "Y displacement", camerapose_targetspace[1]);
      SmartDashboard.putNumber(pCameraName + "Z displacement", camerapose_targetspace[2]);

      SmartDashboard.putNumber(pCameraName + "Roll displacement", camerapose_targetspace[3]);
      SmartDashboard.putNumber(pCameraName + "Pitch displacement", camerapose_targetspace[4]);
      SmartDashboard.putNumber(pCameraName + "Yaw displacement", camerapose_targetspace[5]);
      SmartDashboard.putNumber(pCameraName + "Distance from tag", Math.abs(camerapose_targetspace[0] - (-0.29)));
      
    }
    else
    {
      SmartDashboard.putNumber(pCameraName + "X displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Y displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Z displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Roll displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Pitch displacement", -1);
      SmartDashboard.putNumber(pCameraName + "Yaw displacement", -1);
      
      
    }
  }

  public void logLimelightInfo()
  {
      double[] west = getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.WEST_CAMERA);
      double[] east = getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.EAST_CAMERA);

      if(west != null && east != null)
      {
        SmartDashboard.putNumber("LimeLightWestMinusEast", west[0] - east[0] );
      }
     
      double[] east_botpose = getBotposeForSpecificCamera(LimelightManager.EAST_CAMERA);
      double[] west_botpose = getBotposeForSpecificCamera(LimelightManager.WEST_CAMERA);

      if(west_botpose != null && east_botpose != null)
      {
        SmartDashboard.putNumber("BotposetWestMinusEast", west_botpose[1] - east_botpose[1] );
      }

    logLimelightInfo("East-", east);
    logLimelightInfo("West-", west);
    logLimelightInfo("Combined-", getCameraPoseTargetSpace());
    logLimelightInfo("East-Botpose", east_botpose);
    logLimelightInfo("West-Botpose", west_botpose);
   
  }


}
