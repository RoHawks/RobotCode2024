package universalSwerve.components;

public interface ITranslationSystem {

    void Initialize();

    void SetVelocity(double pVelocity);
    double GetVelocity();
    /*
    Returns in inches
    */
    
    double GetDistanceTravelled();
    void ResetDistanceTravelled();

    void SetToBreakMode();
    void SetToCoastMode();

    void StopEverything();

    double GetPercentOutput();
    double GetCurrentDraw();

    
}
