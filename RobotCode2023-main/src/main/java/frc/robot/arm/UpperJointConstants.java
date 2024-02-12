package frc.robot.arm;

public class UpperJointConstants 
{
     /*
     * For audit purposes, here's what this was after last day at AB before Hofstra
     *     public static double INTAKING = 0.10;
    public static double MID_CONE = 0.414;
    public static double TOP_CONE = 0.578;
    public static double GRAB_LOCATION = 0.068;
    public static double ROTATION_SPOT = 0.21;
     * 
     */
    /*
     * As measured at hofstra practice field on Thusday
     *top cone
        upper 0.630

mid cone
upper 0.413


floor
upper 0.237


cube
mid
upper 0.380

cube high
upper 0.537
     */
    public static double DEBODGE_OFFSET_FROM_TOOTH_SKIP = 0.0556;
    public static double INTAKING = 0.10 + DEBODGE_OFFSET_FROM_TOOTH_SKIP ;
    public static double TOP_CONE = 0.630 + DEBODGE_OFFSET_FROM_TOOTH_SKIP ;
    public static double TOP_CUBE = 0.537 + DEBODGE_OFFSET_FROM_TOOTH_SKIP ; //what we measured on practice field was 0.537 + DEBODGE_OFFSET_FROM_TOOTH_SKIP; //what wehave been using recently is 0.630 + DEBODGE_OFFSET_FROM_TOOTH_SKIP  
    public static double MID_CONE = 0.448  + DEBODGE_OFFSET_FROM_TOOTH_SKIP;
    public static double MID_CUBE = 0.38 + DEBODGE_OFFSET_FROM_TOOTH_SKIP; //measured on practice 0.380 + DEBODGE_OFFSET_FROM_TOOTH_SKIP;  //recent: 0.448 + DEBODGE_OFFSET_FROM_TOOTH_SKIP;
    public static double LOW_GOAL = 0.237 + DEBODGE_OFFSET_FROM_TOOTH_SKIP;
    public static double GRAB_LOCATION = 0.134;
    public static double GRAB_LOCATION_FOR_CUBE = 0.150;  //0.154;
    public static double GRAB_LOCATION_FOR_CUBE_FOR_AUTO_ONLY = 0.140;
    public static double ROTATION_SPOT = 0.21 + DEBODGE_OFFSET_FROM_TOOTH_SKIP;
}
