package org.usfirst.frc.team6560.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// drivetrain motors
	public static final int R1_MOTOR = 8;
    public static final int R2_MOTOR = 9;
    
    public static final int L1_MOTOR = 7;
    public static final int L2_MOTOR = 6;
    
    public static final int ELEVATOR_LEVEL_1_MOTOR = 5;
    public static final int ELEVATOR_LEVEL_1_LIMIT_SWITCH_TOP = 3;
    public static final int ELEVATOR_LEVEL_1_LIMIT_SWITCH_BOTTOM = 2;

     
    public static final int ELEVATOR_LEVEL_2_MOTOR = 4;
    public static final int ELEVATOR_LEVEL_2_LIMIT_SWITCH_TOP = 1;
    public static final int ELEVATOR_LEVEL_2_LIMIT_SWITCH_BOTTOM = 0;
	
	    public static final int GRABBER_SWINGING_MOTOR = 1;
    public static final int GRABBER_BALL_MOTOR = 2;
    public static final int CLIMB_MOTOR = 3;
	
	public static final int CLIMB_EXTENSION_SOLENOID = 1;
    public static final int REAR_HATCH_SHOOTING_SOLENOID = 2;
    public static final int REAR_HATCH_EXTENSION_SOLENOID = 3;
        
	public static class Logitech {
		public static final int ID = 0;

		public static final int TRIGGER = 1;
		public static final int GRIP = 2;
		public static final int BUTTON_3 = 3;
		public static final int BUTTON_4 = 4;
		public static final int BUTTON_5 = 5;
		public static final int BUTTON_6 = 6;
		public static final int BUTTON_7 = 7;
		public static final int BUTTON_8 = 8;
		public static final int BUTTON_9 = 9;
		public static final int BUTTON_10 = 10;
		public static final int BUTTON_11 = 11;
	}
	
	public static class Xbox {
		public static final int ID = 1;
		
		public static final int LEFT_JOY_X = 0;
		public static final int LEFT_JOY_Y = 1;
		
		public static final int RIGHT_JOY_X = 4;
		public static final int RIGHT_JOY_Y = 5;

		public static final int LEFT_BUMPER = 5;
		public static final int RIGHT_BUMPER = 6;

		public static final int BUTTON_Y = 4;
		public static final int BUTTON_X = 3;
		public static final int BUTTON_B = 2;
		public static final int BUTTON_A = 1;
	}
}
