package universalSwerve.hardware;

public interface IWheelAngleReader 
{
    /*
    See CoordinateSystem.txt for details!
    The angle here refers to the angle of the wheel relative to the the frame of the robot.
    So 0 degrees means that the wheel is pointed towards the "front" of the robot;
    */
    double GetCurrentAngle();
    
    //Call this at startup.  
    void Initialize();
     
}
