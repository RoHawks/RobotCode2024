package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightInformation {

    private String mName;
    private String mID;
    private String mHostname;

    public LimelightInformation(String pName, String pID)
    {
        mName = pName;
        mID = pID;
        mHostname = "limelight-" + pID;
    }

    private double[] mCameraPoseTargetSpace;


    public void calculateCameraPoseTargetSpace(double pHorizontalOffsetMeters)
    {
        try
        {
            NetworkTable table = NetworkTableInstance.getDefault().getTable(mHostname);
           
            //NetworkTableEntry jsonNTE = table.getEntry("json");
           
            //String json_dump = jsonNTE.getString("null");
           
            
            //double[] camerapose_targetspace = getCameraPoseTargetSpace(json_dump);
            double[] camerapose_targetspace = table.getValue("camerapose_targetspace").getDoubleArray();
            SmartDashboard.putNumber("hereD", System.currentTimeMillis());

            if (camerapose_targetspace != null)
            {
                camerapose_targetspace[0] += pHorizontalOffsetMeters;
                mCameraPoseTargetSpace = camerapose_targetspace;
            }
            else
            {
                mCameraPoseTargetSpace = null;
            }
            SmartDashboard.putNumber("Lime: tA", table.getEntry("ta").getDouble(0));

            SmartDashboard.putNumber("Lime: tZ", mCameraPoseTargetSpace[2]);

        }
        catch (Exception e)
        {
            mCameraPoseTargetSpace = null;
        }
    }


    /* */
    private static double[] getCameraPoseTargetSpace(String json)
    {
        SmartDashboard.putString("json", json);
       try {
        int idx1 = json.indexOf("t6c_ts");
        int idx2 = json.indexOf("t6r_fs");
        if (idx1 > 0 && idx2 > 0)
        {
          // SmartDashboard.putString("test", json.substring(idx1,idx2));
          String s1 = json.substring(idx1,idx2);
          s1 = s1.substring(9,s1.length()-3);
          String[] comma_split_list = s1.split(",");
          double[] double_array = new double[6];
          for (int i = 0; i < comma_split_list.length; i++)
          {
              // System.out.println(comma_split_list[i]);
              double_array[i] = Double.parseDouble(comma_split_list[i]);
          }
          // SmartDashboard.putString("should work", Arrays.toString(double_array));
          return double_array;
        }
        return null;
       }
       catch (Exception e) {
        SmartDashboard.putString("cameraPostException", e.getLocalizedMessage());
        return null;
       }
      
    }
    

    public double[] getCameraPoseTargetSpace()
    {
        return mCameraPoseTargetSpace;
    }
}
