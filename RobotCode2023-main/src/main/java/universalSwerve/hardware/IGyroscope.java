package universalSwerve.hardware;

public interface IGyroscope 
{
    /*
        See CoordinateSystem.txt for info!
        The angle here refers to which direction the "front" of the robot refers to relative to the direction
        The the driver is facing (i.e., straight ahead on the field is 0 degrees.)
    */
    double GetCurrentAngle();    
    
    void SetCurrentAngle(double pAngle);
    
}
