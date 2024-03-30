package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceInfo 
{
 
    private Alliance mAlliance = null;

    private static AllianceInfo mInstance = new AllianceInfo();

    public static AllianceInfo GetInstance()
    {
        return mInstance;
    }

    public void LoadAllianceFromFMS()
    {
        mAlliance = DriverStation.getAlliance().get();
    }

    public String GetName()
    {
        return mAlliance.name();
    }

    public double GetCentralAprilTagDistanceFromDriversRightWall()
    {
        if(mAlliance == null || mAlliance == Alliance.Blue)
        {
            return 5.54776; //meters
        }
        else
        {
            return 2.66348;
        }
    }

    public double GetLowGoalRotation()
    {
        return mAlliance != null && mAlliance == Alliance.Red ? 270.0 : 90.0 ;
    }

    public boolean ShouldFlipAutos()
    {
        return mAlliance != null && mAlliance == Alliance.Red;
    }

    public String GetBotposeTableName()
    {
        return mAlliance != null && mAlliance == Alliance.Red ? "botpose_wpired" : "botpose_wpiblue" ;
        
    }

    public void OverrideAllianceForTestingPurposes(Alliance pAlliance)
    {
        
        mAlliance = pAlliance;
    }

}
