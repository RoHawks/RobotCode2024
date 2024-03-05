package frc.robot;

public class LimelightManager 
{
    private LimelightInformation[] mLimeLights;
    private CameraBlendMethod mCameraBlendMethod = CameraBlendMethod.Prioritize;

    
    public static final int WEST_CAMERA = 0;
    public static final int EAST_CAMERA = 1;

    public LimelightManager()
    {
        mLimeLights = new LimelightInformation[2];
        mLimeLights[WEST_CAMERA] = new LimelightInformation("west", "west");
        mLimeLights[EAST_CAMERA] = new LimelightInformation("east", "east");
    }

    public void calculateCameraPoseTargetSpace()
    {
        for(int i = 0; i < mLimeLights.length; i++)
        {
            mLimeLights[i].calculateCameraPoseTargetSpace(
                i == WEST_CAMERA ? 0.532 : 0                //this is 22.5 incvhes plus a tiny fuidge factor from experimetnation
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

}
