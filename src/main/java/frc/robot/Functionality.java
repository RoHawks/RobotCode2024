package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Functionality {
    public static void configureSparkMaxCustomizable(CANSparkMax pSparkMax, int pCurrentLimit, double pOpenLoopRampRate)
    {
        pSparkMax.restoreFactoryDefaults(); 
        pSparkMax.setSmartCurrentLimit(pCurrentLimit);    
        pSparkMax.setOpenLoopRampRate(pOpenLoopRampRate);
        pSparkMax.burnFlash();
    }


    public static void configureSparkMaxWeak(CANSparkMax pSparkMax)
    {
        pSparkMax.restoreFactoryDefaults(); 
        pSparkMax.setSmartCurrentLimit(20);    
        pSparkMax.setOpenLoopRampRate(0.2);
        pSparkMax.burnFlash();
    }
    
    public static void configureSparkMaxStrong(CANSparkMax pSparkMax)
    {
        pSparkMax.restoreFactoryDefaults(); 
        pSparkMax.setSmartCurrentLimit(40);    
        pSparkMax.setOpenLoopRampRate(0.2);
        pSparkMax.burnFlash();
    }

    public static double[] getCameraPoseTargetSpace(String json)
    {
      // try {
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
      // }
      // catch (Exception e) {
      //   throw e;
       
      // }
      
    }

    public static double[] getLimeLightInfo()
    {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry jsonNTE = table.getEntry("json");
      String json_dump = jsonNTE.getString("null");

      double[] camerapose_targetspace = Functionality.getCameraPoseTargetSpace(json_dump);
      return camerapose_targetspace;
    }
    

}
