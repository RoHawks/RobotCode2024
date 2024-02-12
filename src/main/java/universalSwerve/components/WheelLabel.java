package universalSwerve.components;

public final class WheelLabel 
{
    private String mTextLabel;

    public static final WheelLabel NE = new WheelLabel("NE");
    public static final WheelLabel SE = new WheelLabel("SE");
    public static final WheelLabel SW = new WheelLabel("SW");
    public static final WheelLabel NW = new WheelLabel("NW");
    

    private WheelLabel(String pTextLabel)
    {
        mTextLabel = pTextLabel;
    }

    public String Text()
    {
        return mTextLabel;
    }

    public static WheelLabel FromInt(int pIndex)
    {
            switch (pIndex)
            {
                case 0:
                    return NE;
                case 1:
                    return SE;
                case 2:
                    return SW;
                case 3:
                    return NW;
                default:
                    throw new RuntimeException("Unknown wheel label index " + Integer.toString(pIndex));
            }    

    }
}
