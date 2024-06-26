package universalSwerve.utilities;

public class PIDFConfiguration
{ 

    private double mP;
    private double mI;
    private double mD;
    private double mF;
    private double mS;


    public PIDFConfiguration(double pP, double pI, double pD, double pF)
    {
        mP = pP;
        mI = pI;
        mD = pD;
        mF = pF;
        mS = 0;
    
    }

    public PIDFConfiguration(double pP, double pI, double pD, double pF, double pS)
    {
        mP = pP;
        mI = pI;
        mD = pD;
        mF = pF;
        mS = pS;
    }



    public double P()
    {
        return mP;
    }

    public double I()
    {
        return mI;
    }

    public double D()
    {
        return mD;
    }

    public double F()
    {
        return mF;
    }

    public double S()
    {
        return mS;
    }

}
