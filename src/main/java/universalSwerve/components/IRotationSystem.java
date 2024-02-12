package universalSwerve.components;

public interface IRotationSystem 
{
    /*
        This will get called once when the robot starts
    */
    void Initialize();

    /*
    Gets the current angle, see CoordinateSystem.txt for details
    */
    double GetCurrentAngle();

    /* if there is an abs encoder used at startup, return it here.  Just for debugging. */
    double GetRawCurrentAngle();


    /*
    Sets the target angle, see CoordinateSystem.txt for details
    */
    void SetAngle(double pTargetAngle);

    void StopEverything();
    
}
