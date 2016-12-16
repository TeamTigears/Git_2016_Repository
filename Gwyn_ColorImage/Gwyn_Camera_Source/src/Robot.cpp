#include "WPILib.h"
#include <stdlib.h>
#include <Relay.h>
#include <stdio.h>
#define COMPILED_TIME "19:13 30-Mar-2015"

// Conditional compile options


#define ECLIPSE // Comment out if using WindRiver
//#define USE_HEIGHT_LIMIT_SWITCHES


//TODO: for Madera REPLACE Y PWM CABLE ON RELAY 1: FOR GRIPPER TWIST

// for the ability to switch between arcade and tank

//#define DISABLE_TOTE_HEIGHT_USE

// switch box #defines
#define HEIGHT_LIFT_MOVE 2
#define ARCADE_SWITCH 6
#define GAME_PAD_SWITCH 7
#define USE_TOTE_PRESSENCE_SENSOR 8
#define REVERSE_DIR_DRIVE 9
#define AUTONOMOUS_WITH_ENCODER 10
#define AUTONOMOUS_SWITCH_ZERO 11
#define AUTONOMOUS_SWITCH_ONE 12

//initial states
//#define GRIP_START_CLOSED

// right stick arcade drive #defines
#define CRATE_DOWN 2
#define CRATE_UP 3
//#define CANCEL_CRATE_MOVE 7
#define GRIPPER_FORWARD_ROTATION 6
#define GRIPPER_BACKWARD_ROTATION 7
#define GRIP_TOGGLE 1
#define GRIP_OPEN 11
#define GRIP_CLOSE 10
#define GRIPPER_OUT 5
#define GRIPPER_IN 4

//Lifter heights
#define BOTTOM_TOTE 3
#define TOTE_OVER_SCORING_PLATFORM 10
#define TOTE_OVER_STEP 11
#define PICK_GARBAGE_CAN 12
#define TOTE_OVER_TOTE 16
#define GARBAGE_CAN_OVER_TOTE 27

//Buttonbox #defines
#define BOTTOM_TOTE_BUTTON 11
#define TOTE_OVER_TOTE_BUTTON 7
#define TOTE_OVER_SCORING_PLATFORM_BUTTON 8
#define GARBAGE_CAN_OVER_TOTE_BUTTON 5
#define PICK_GARBAGE_CAN_BUTTON 2
#define TOTE_OVER_STEP_BUTTON 3
#define COMPRESSOR_START 4
#define COMPRESSOR_STOP 6
#define INCREMENT_LED 9
#define DECREMENT_LED 10

// right stick/gamepad #defines
#define GAMEPAD_CRATE_UP 0
#define GAMEPAD_DOWN 180 //Use the joystick POV (d-pad)
#define GAMEPAD_COMPRESSOR_START 10
#define GAMEPAD_COMPRESSOR_STOP 9
#define GAMEPAD_INCREMENT_LED 6
#define GAMEPAD_DECREMENT_LED 5
#define GAMEPAD_GRIP_TOGGLE 2
#define GAMEPAD_GRIP_OPEN 1
#define GAMEPAD_GRIP_CLOSE 3
#define GAMEPAD_LEFT_X 0
#define GAMEPAD_LEFT_Y 1
#define GAMEPAD_RIGHT_X 4
#define GAMEPAD_RIGHT_Y 5

#define LIFT_TO_TOP 1
#define LIFT_TO_MIDDLE
#define LIFT_TO_BOTTOM -1


//Autonomous modes
#define AUTONOMOUS_DRIVE_FORWARD_STATE 3
#define AUTONOMOUS_PICK_UP_BIN_AND_PUSH_TOTE_STATE 0
#define AUTONOMOUS_PICK_UP_STATE 1
#define AUTONOMOUS_PUSH_TOTE_STATE 2
//#define AUTONOMOUS_EXTEND_ARM_STATE 3

//analog distance #defines
#define SLOPE_TO_FIND_DISTANCE_APART 0.0487
#define NUMBER_TO_ADD_NEEDED_TO_FIND_DISTANCE_APART 1.6814
#define LOWEST_NUMBER_FOR_DISTANCE_ON_LINEAR_GRAPH 231
#define HIGHEST_NUMBER_FOR_DISTANCE_ON_LINEAR_GRAPH 1890
#define TOTE_SENSING_THRESHOLD 14//TODO check value of sensor
//state machine states for picking up tote in autonomous for AUTONOMOUS_PICK_UP_STATE

typedef enum RelayDirection {
	OFF = 0,
	FORWARD = 1,
	REVERSE = 2
} RelayDirection;

enum auto_pick_up_totes_states{
	AUTO_PICK_UP_GOTO_BIN_HEIGHT = 0,
	AUTO_PICK_UP_BIN_CASE = 1,
	AUTO_PICK_UP_GO_UP_AFTER_GRIP_RECYCLE_BIN = 2,


	AUTO_PICK_UP_DRIVE_TO_CONTAINER_CASE = 3,
	AUTO_PICK_UP_STOP_DRIVE_TO_CONTAINER_CASE = 4,
	AUTO_PICK_UP_PLACE_RECYCLE_CASE = 5,
	AUTO_PICK_UP_START_GRAB_TOTE_CASE= 6,
	AUTO_PICK_UP_FINISH_GRAB_TOTE_CASE =7,
	AUTO_PICK_UP_LIFT_TO_TOP_CASE =8,
	AUTO_PICK_UP_TURN_LEFT_CASE = 9,
	AUTO_PICK_UP_STOP_TURN_LEFT_CASE = 10,
	AUTO_PICK_UP_DRIVE_FORWARD_STATE = 11,
	AUTO_PICK_UP_STOP_DRIVE_FORWARD_STATE = 12,
};

enum auto_pick_bin_push_totes
{
	AUTO_SETUP = 0,
	AUTO_GET_TO_BIN_HEIGHT = 1,
	AUTO_GRAB_BIN = 2,
	AUTO_GO_UP_AFTER_GRAB_BIN = 3,
	AUTO_DRIVING_FORWARD = 4,
	AUTO_STOP_DRIVING_FORWARD = 5,
	AUTO_TURNING_LEFT = 6,
	AUTO_STOP_TURNING_LEFT = 7,
	AUTO_DRIVING_TO_AUTO_ZONE = 8,
	AUTO_STOP_DRIVING_TO_AUTO_ZONE = 9,
	AUTO_STOP=10,
};



enum controller_states
{
	LEFT_STICK,
	GAMEPAD,
};




#define GRIPPER_OPEN DoubleSolenoid::kReverse
#define GRIPPER_CLOSE DoubleSolenoid::kForward

#define CRATE_MOTOR_UP Relay::kForward
#define CRATE_MOTOR_DOWN Relay::kReverse
#define CRATE_MOTOR_OFF Relay::kOff

//#define AUTONOMOUS_PICK_UP_CASE 0
//#define AUTONOMOUS_PICK_UP_DRIVE_TO_CONTAINER_CASE 1
//#define AUTONOMOUS_PICK_UP_STOP_DRIVE_TO_CONTAINER_CASE 2
//#define PLACE_RECYCLE_AND_GRAB_CASE 3
//#define AUTONOMOUS_TURN_LEFT_CASE 4
//#define AUTONOMOUS_STOP_TURN_LEFT_CASE 5
//#define PICK_UP_AUTONOMOUS_DRIVE_FORWARD_STATE 6
//#define PICK_UP_AUTONOMOUS_STOP_DRIVE_FORWARD_STATE 7

#define AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_CASE 0
#define AUTONOMOUS_PUSH_TOTE_STOP_DRIVE_TO_CONTAINER_CASE 1




#define AUTONOMOUS_TURN_RIGHT_CASE 7
#define AUTONOMOUS_STOP_TURN_RIGHT_CASE 8
#define AUTONOMOUS_DRIVE_HORIZONTAL_STATE 9
#define AUTONOMOUS_STOP_DRIVE_HORIZONTAL_STATE 10
#define SECOND_AUTONOMOUS_TURN_RIGHT_CASE 11
#define SECOND_AUTONOMOUS_STOP_TURN_RIGHT_CASE 12
#define SECOND_AUTONOMOUS_DRIVE_FORWARD_STATE 13
#define SECOND_PICK_UP_AUTONOMOUS_STOP_DRIVE_FORWARD_STATE 14
#define SECOND_PLACE_RECYCLE_AND_GRAB_STATE 15
#define AUTONOMOUS_DRIVE_BACKWARD_STATE 16
#define AUTONOMOUS_STOP_DRIVE_BACKWARD_STATE 17

// turning/moving limits in autonomous for AUTONOMOUS_PICK_UP_STATE
#define AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS 1000
#define AUTONOMOUS_PICK_UP_STATE_STOP_LIFT_TIME 4
#define AUTONOMOUS_PICK_UP_STATE_STOP_DRIVE_TO_CONTAINER_TIME 2
#define AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTOZONE_TIME 5
#define AUTONOMOUS_DRIVE_FORWARD_TO_AUTOZONE_IN_PICKING_UP_TOTES_IN_AUTONOMOUS 5
#define AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS 1000
#define AUTONOMOUS_PICK_UP_STATE_MOVE_HORIZONTAL_CLICKS 1000
#define AUTONOMOUS_PICK_UP_STATE_MOVE_HORIZONTAL_TIME 1
#define AUTONOMOUS_TURN_NINETY_CLICKS 2000
#define AUTONOMOUS_TURN_NINETY_CLICKS_TIME 1

//pick bin autonomous limits
#define AUTONOMOUS_PICK_BIN_STATE_MOVE_TO_AUTO_ZONE_CLICKS 1000
#define AUTONOMOUS_PICK_BIN_STATE_MOVE_TO_AUTOZONE_TIME 5

//state machine states for the AUTONOMOUS_PUSH_TOTE_STATE
#define AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_CASE 0
#define AUTONOMOUS_PUSH_TOTE_STOP_DRIVE_TO_CONTAINER_CASE 1
#define AUTONOMOUS_LOWER_LIFT_CASE 2
#define AUTONOMOUS_GRAB_CONTAINER_CASE 3
#define AUTONOMOUS_TURN_CASE 4
#define AUTONOMOUS_STOP_TURN_CASE 5
#define AUTONOMOUS_DRIVE_FORWARD_CASE 6
#define AUTONOMOUS_STOP_DRIVE_FORWARD_CASE 7


//encoder values/powers for AUTONOMOUS_PUSH_TOTE_STATE

#define AUTONOMOUS_PUSH_TOTE_TURN_CONSTANT .5
#define AUTONOMOUS_PUSH_TOTE_TURN_CLICKS 200
#define AUTONOMOUS_PUSH_TOTE_TURN_TIME 5
#define AUTONOMOUS_PUSH_TOTE_DRIVE_FORWARD_STATE_CLICKS 300
#define AUTONOMOUS_PUSH_TOTE_DRIVE_FORWARD_TIME 5
#define AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_STATE_CLICKS 300
#define AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_TIME 5

//extend arm/ drive forward autonomous encoder/time values
#define AUTONOMOUS_BACKWARD_CLICKS 100
#define AUTONOMOUS_BACKWARD_TIME 5
//#define AUTONOMOUS_DRIVE_FORWARD_TIME 2.5
#define AUTONOMOUS_DRIVE_FORWARD_TIME 1.5
#define AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS 300


#define LEFT_AUTONOMOUS_DRIVE_POWER 1
#define RIGHT_AUTONOMOUS_DRIVE_POWER 1

//constants
#define ARCADE_DRIVE 0
#define TANK_DRIVE 1
#define GRIPPER_OPENED 0
#define GRIPPER_CLOSED 1
#define CURRENT_ROBOT 0
#define ARIEL 1
#define ASIMOV 2
#define DOROTHY 3

#ifdef ECLIPSE
#define TALON_LEFT 0
#define TALON_RIGHT 1
#else
#define TALON_LEFT 1
#define TALON_RIGHT 2
#endif
#define PROPORTIONAL_CONSTANT 20
#define LEFT_DRIVE_ENCODER_BACKLASH 5
#define RIGHT_DRIVE_ENCODER_BACKLASH 5


//Digital IOs
#define ROBOT_MODE_BIT_0 13
#define ROBOT_MODE_BIT_1 14

//for the color bit its the color bit -10 on the MXP expansion board
#ifdef ECLIPSE
#define COLOR_BIT_0 17
#define COLOR_BIT_1 18
#define COLOR_BIT_2 19
#define ARD_INPUT_0 20
#define ARD_INPUT_1 21
#define ARD_INPUT_2 22
#define ARD_INPUT_3 23
#define ARD_INPUT_4 24
#define ARD_LIVE_SIGNAL_BIT 25
#else
#define ARD_INPUT_0 7
#define ARD_INPUT_1 8
#define ARD_INPUT_2 9
#define ARD_INPUT_3 10
#define ARD_INPUT_4 11
#define ARD_LIVE_SIGNAL_BIT 12
#endif

#define ARD_CYCLE_LIMIT 0

//#define ARD_INPUT_0 7
//#define ARD_INPUT_1 8
//#define ARD_INPUT_2 9
//#define ARD_INPUT_3 1
//#define ARD_INPUT_4 1
//#define ARD_INPUT_5 1


#define LEFT_ENCODER_FIRST_CHANNEL 3
#define LEFT_ENCODER_SECOND_CHANNEL 4
#define RIGHT_ENCODER_FIRST_CHANNEL 5
#define RIGHT_ENCODER_SECOND_CHANNEL 6

//#define BOTTOM_SENSOR_CHANNEL 7
//#define MIDDLE_SENSOR_CHANNEL 8

#define COMPRESSOR_PCM 0
#define COMPRESSOR_RELAY 1
#define PRESSURE_SENSOR 2
#define elif else if

class Robot: public IterativeRobot
{

	RobotDrive myRobot; // robot drive system
	Joystick right_stick; // right stick or gamepad
	Joystick left_stick;// left stick
	Joystick switch_box;
	Joystick game_pad;
	Joystick button_box;
#ifdef ECLIPSE
	PowerDistributionPanel power_board;
#else
	DriverStationLCD *driveLCD;
#endif

#ifdef ECLIPSE
	DigitalOutput arduino_light0;
	DigitalOutput arduino_light1;
	DigitalOutput arduino_light2;
#endif
	DigitalInput arduino_input_bit0;
	DigitalInput arduino_input_bit1;
	DigitalInput arduino_input_bit2;
	DigitalInput arduino_input_bit3;
	DigitalInput arduino_input_bit4;
#ifdef ECLIPSE
	DigitalOutput arduino_signal_bit;
#endif

	int ard_output_cycle_counter;
	bool ard_live_signal_value;

	DigitalInput robot_mode0;
	DigitalInput robot_mode1;


	DigitalInput top_limit_switch;
	DigitalInput bottom_limit_switch;




	//	DigitalOutput output_test_10;
	//	DigitalOutput output_test_11;
	//	DigitalOutput output_test_12;
	//	DigitalOutput output_test_13;
	//	DigitalOutput output_test_14;
	//	DigitalOutput output_test_15;


	Encoder right_encoder;
	Encoder left_encoder;
	Timer timer;
	int32_t current_left_location;
	int32_t current_right_location;
	int32_t stop_left_location;
	int32_t stop_right_location;


	int autoLoopCounter;
	int right_encoder_starter_value;
	int left_encoder_starter_value;
	float timer_starting_position;
	int drive_mode;
	int controller_drive_mode;
	float left_wheel_speed;
	float right_wheel_speed;
	float prev_left_wheel_speed;
	float prev_right_wheel_speed;
	float left_teleop_motor_power;
	float right_teleop_motor_power;
	bool grip_is_open;
	bool grip_pressed;
	bool grip_prev_pressed;
	bool grip_extend;
	float left_autonomous_motor_power;
	float right_autonomous_motor_power;
	int left_encoder_value;
	int right_encoder_value;
	int auto_state;
	int auto_mode;
	bool auto_with_encoder;
	int invert_left;
	int invert_right;
	bool invert_lift;
	int reverse_direction;

	int robo_num;

	Relay crate_lifting_motor;
	Relay gripper_rotation_motor;

	RelayDirection crate_motor_direction;

	Compressor compressor;

	DoubleSolenoid piston_control_solenoid;
	DoubleSolenoid gripper_control_solenoid;
	DoubleSolenoid gripper_extension_solenoid;

#ifdef ECLIPSE
	AnalogInput tote_sensor;
#else
	AnalogChannel tote_sensor;
#endif

	LiveWindow *lw;

	int current_tote_height;
	int target_tote_height;

	bool use_tote_height;
	bool up_previously_pressed;
	bool down_previously_pressed;
	bool target_button_pressed;
	bool got_to_tote_target;
	RelayDirection prev_crate_lifter_dir;

	// TODO: add variable to indicate direction that lift mechanism should go

	int distance_using_tote_sensor;
	bool has_tote;
	bool limit_switch_check;
	int previous_lifter_height;

	uint8_t led_value;
	bool increment_color_flag;
	char dashboard_output1[30];
	char dashboard_output2[30];
	char dashboard_output3[30];
	char dashboard_output4[30];

public:
	Robot() :
#ifdef ECLIPSE

		myRobot(0, 1),	// these must be initialized in the same order
		//	right_stick(0, 3, 11),	// as they are declared above.
		right_stick(0),
		left_stick(1),
		switch_box(2),
		game_pad(1),
		button_box(3),
#else
		myRobot(1, 2),	// these must be initialized in the same order
		right_stick(1),	// as they are declared above.
		left_stick(2),
		switch_box(3),
		game_pad(2),
		button_box(4),
#endif
#ifdef ECLIPSE
		power_board(),
#endif
		//		arduino_light0(0),
		//		arduino_light1(1),
		//		arduino_light2(2),

#ifdef ECLIPSE
		arduino_light0(COLOR_BIT_0),
		arduino_light1(COLOR_BIT_1),
		arduino_light2(COLOR_BIT_2),
#endif

		arduino_input_bit0(ARD_INPUT_0),
		arduino_input_bit1(ARD_INPUT_1),
		arduino_input_bit2(ARD_INPUT_2),
		arduino_input_bit3(ARD_INPUT_3),
		arduino_input_bit4(ARD_INPUT_4),
#ifdef ECLIPSE
		arduino_signal_bit(ARD_LIVE_SIGNAL_BIT),
#endif

		ard_output_cycle_counter(0),
		ard_live_signal_value(false),

		robot_mode0(ROBOT_MODE_BIT_0),
		robot_mode1(ROBOT_MODE_BIT_1),
#ifdef ECLIPSE
		top_limit_switch(0),
#else
		top_limit_switch(ARD_INPUT_5),
#endif
		bottom_limit_switch(2),

		//		top_limit_switch(11),
		//		middle_limit_switch(12),
		//		bottom_limit_switch(13),

		//		output_test_10(10),
		//		output_test_11(11),
		//		output_test_12(12),
		//		output_test_13(13),
		//		output_test_14(14),
		//		output_test_15(15),

		// TODO: initialize encoders here

		// Digital Inputs
		//k4x = 1440 clicks per revolution
		//gear ratio is 18:1
		right_encoder(RIGHT_ENCODER_FIRST_CHANNEL,RIGHT_ENCODER_SECOND_CHANNEL),
		left_encoder(LEFT_ENCODER_FIRST_CHANNEL,LEFT_ENCODER_SECOND_CHANNEL),

		timer(),

		robo_num(0),

#ifdef ECLIPSE
		crate_lifting_motor(0, Relay::kBothDirections),
		gripper_rotation_motor(1, Relay::kBothDirections),

		crate_motor_direction(OFF),

		compressor(COMPRESSOR_PCM),
		piston_control_solenoid(4,5),
		gripper_control_solenoid(0,1),
		gripper_extension_solenoid(2,3),
#else
		crate_lifting_motor(1, Relay::kBothDirections),
		gripper_rotation_motor(2, Relay::kBothDirections),
		compressor(COMPRESSOR_RELAY,PRESSURE_SENSOR),
		piston_control_solenoid(5,6),
		gripper_control_solenoid(1,2),
		gripper_extension_solenoid(3,4),
#endif



		// Analog Inputs
		tote_sensor(1),

		lw(NULL),

		// primitives
		current_left_location(0),
		current_right_location(0),

		stop_left_location(0),
		stop_right_location(0),

		autoLoopCounter(0),

		right_encoder_starter_value(0),
		left_encoder_starter_value(0),

		timer_starting_position(0),


		drive_mode(ARCADE_DRIVE),
		controller_drive_mode(LEFT_STICK),

		left_wheel_speed(0),
		right_wheel_speed(0),

		prev_left_wheel_speed(0),
		prev_right_wheel_speed(0),

		left_teleop_motor_power(0),
		right_teleop_motor_power(0),

#ifdef GRIPPER_START_CLOSED
		grip_is_open(false),
#else
		grip_is_open(true),
#endif
		grip_pressed(false),
		grip_prev_pressed(false),
		grip_extend(false),

		left_autonomous_motor_power(0),
		right_autonomous_motor_power(0),

		left_encoder_value(0),
		right_encoder_value(0),

		auto_state(0),
		auto_mode(0),
		auto_with_encoder(false),

		invert_left(1),
		invert_right(1),
		invert_lift(false),
		reverse_direction(1),

		current_tote_height(1),
		target_tote_height(-1),
		use_tote_height(true),

		up_previously_pressed(false),
		down_previously_pressed(false),

		target_button_pressed(false),
		got_to_tote_target(true),
		prev_crate_lifter_dir(OFF),

		distance_using_tote_sensor(0),
		has_tote(false),
		limit_switch_check(true),
		previous_lifter_height(1),

		led_value(1),
		increment_color_flag(false)
{
		myRobot.SetExpiration(0.1);
}

private:
	void RobotInit()
	{
		printf("Compiled on %s\n",  COMPILED_TIME);
		lw = LiveWindow::GetInstance();
		robo_num = robot_mode0.Get()+(robot_mode1.Get()<<1);
		robo_num = DOROTHY;
		switch(robo_num)
		{
		case CURRENT_ROBOT:
			printf("Robot = Current Robot\n");
			invert_left = -1;
			invert_right = 1;
			invert_lift = true;
			break;
		case ARIEL:
			printf("Robot = Ariel\n");
			invert_left = 1;
			invert_right = 1;
			break;
		case ASIMOV:
			printf("Robot = Asimov\n");
			invert_left = 1;
			invert_right = -1;
			break;
		case DOROTHY:
			printf("Robot = Dorothy\n");
			invert_left = -1;
			invert_right = 1;
			break;

		}
		compressor.Start();
		printf("compressor on");

		has_tote = false;
		previous_lifter_height = 0;
	}

	void DisabledInit()
	{
		printf("RobotDisabled\n");
		SmartDashboard::PutString("DB/String 3", "Disabled Init");
		led_value = 1;
	}

	void DisabledPeriodic()
	{
		SmartDashboard::PutString("DB/String 3", "Disabled periodic");
		target_tote_height = -1;
		target_button_pressed = false;
		ArduinoLightDisplay(led_value);
		ArdWatchDogOutput();
	}

	void AutonomousInit()
	{

		left_autonomous_motor_power = 0;
		right_autonomous_motor_power = 0;
		auto_state = 0;
		compressor.Start();
		printf("Compiled on %s\n",  COMPILED_TIME);
#ifdef GRIPPER_START_CLOSED
		grip_is_open=false;
		gripper_control_solenoid.Set(GRIPPER_OPEN);
#else
		grip_is_open=true;
		// has_tote = false;
		gripper_control_solenoid.Set(GRIPPER_OPEN);
		gripper_extension_solenoid.Set(DoubleSolenoid::kReverse);
#endif

		crate_lifting_motor.Set(CRATE_MOTOR_OFF); // changed from Relay::kForward
		//		if (top_limit_switch.Get())
		//		{
		//			crate_lifting_motor.Set(Relay::kOff);
		//			printf("crate lifter at top\n");
		//			auto_state++;
		//
		//		}
		printf("gripper open\n");
		autoLoopCounter = 0;
#ifdef ECLIPSE
		current_tote_height = GetDigitalInputValue4470(
				arduino_input_bit0,arduino_input_bit1,arduino_input_bit2,
				arduino_input_bit3,arduino_input_bit4);
#else
		current_tote_height = GetDigitalInputValue4470(
				arduino_input_bit0,arduino_input_bit1,arduino_input_bit2,
				arduino_input_bit3,arduino_input_bit4, 0);
#endif


		auto_mode = (switch_box.GetRawButton(AUTONOMOUS_SWITCH_ZERO)?1:0)+(switch_box.GetRawButton(AUTONOMOUS_SWITCH_ONE)?2:0);
		auto_with_encoder= switch_box.GetRawButton(AUTONOMOUS_WITH_ENCODER)?1:0;

#ifdef ECLIPSE

		SmartDashboard::PutString("DB/String 0", "");
		SmartDashboard::PutString("DB/String 1", "");
		SmartDashboard::PutString("DB/String 2", "");
		SmartDashboard::PutString("DB/String 3", "");
		SmartDashboard::PutString("DB/String 4", "");
		SmartDashboard::PutString("DB/String 5", "");
		SmartDashboard::PutString("DB/String 6", "");
		SmartDashboard::PutString("DB/String 7", "");
		SmartDashboard::PutString("DB/String 8", "");
		SmartDashboard::PutString("DB/String 9", "");

#else
		DriverStationLCD *driveLCD = DriverStationLCD::GetInstance();
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line1, "current tote height %d",current_tote_height);
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line4, "");
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line5, "");
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line6, "");
		driveLCD->UpdateLCD();
#endif
		timer.Start();
		timer_starting_position = timer.Get();
		printf("MAX_VALUE %X",RAND_MAX);
	}

	void AutonomousPeriodic()
	{
		float leftFinalPower;
		float rightFinalPower;
		//TODO: Set to random value to test for errors
		left_encoder_value=left_encoder.GetRaw();
		right_encoder_value=right_encoder.GetRaw();
		//		left_encoder_value = rand();
		//		right_encoder_value= rand();
#ifdef ECLIPSE
		sprintf( dashboard_output2,"left enc: %d",left_encoder_value);
		SmartDashboard::PutString("DB/String 1", dashboard_output2);
		sprintf( dashboard_output2,"right enc: %d",right_encoder_value);
		SmartDashboard::PutString("DB/String 2", dashboard_output2);
		sprintf( dashboard_output2,"auto_mode: %d",auto_mode);
		SmartDashboard::PutString("DB/String 7", dashboard_output2);
		current_tote_height = GetDigitalInputValue4470(
				arduino_input_bit0,arduino_input_bit1,arduino_input_bit2,
				arduino_input_bit3,arduino_input_bit4);
#else

		current_tote_height = GetDigitalInputValue4470(
				arduino_input_bit0,arduino_input_bit1,arduino_input_bit2,
				arduino_input_bit3,arduino_input_bit4, 0);
		DriverStationLCD *driveLCD = DriverStationLCD::GetInstance();
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line1, "current tote height %d",current_tote_height);
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line2,"left enc: %d", left_encoder_value);
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line3,"right enc: %d", right_encoder_value);
#endif

		//printf("Current Tote Height %d\n",current_tote_height);
		// auto_mode = (switch_box.GetRawButton(AUTONOMOUS_SWITCH_ZERO)?1:0)+(switch_box.GetRawButton(AUTONOMOUS_SWITCH_ONE)?2:0);

		ArdWatchDogOutput();

		// TODO: check switchbox switch to determine if encoders can be used

		switch(auto_mode)
		{
		case AUTONOMOUS_PICK_UP_BIN_AND_PUSH_TOTE_STATE: // primary auto mode
			pickBinAndPushTotes();
			break;
		case AUTONOMOUS_PICK_UP_STATE:
			pickingUpTotesInAutonomous();
			break;
		case AUTONOMOUS_PUSH_TOTE_STATE:
			pushingTotesInAutonomous();
			break;
		case AUTONOMOUS_DRIVE_FORWARD_STATE:// get stuff off step
			//			printf("case 0 in auto_mode");
			drivingForwardInAutonomous();
			// switch(auto_state)
			break;
		default:
			extendingArmInAutonomous();
			break;
		}// end of the auto_mode switch statement in autonomous
		//printf("invert_right: %d",invert_right);
		//printf("invert_left: %d",invert_left);
		//printf("right_autonomous_motor_power: %f",right_autonomous_motor_power);
		//printf("left_autonomous_motor_power: %f",left_autonomous_motor_power);
		rightFinalPower = invert_right * right_autonomous_motor_power;
		leftFinalPower = invert_left * left_autonomous_motor_power;
		myRobot.SetLeftRightMotorOutputs(leftFinalPower,rightFinalPower);
		ArduinoLightDisplay(led_value);// has tote and gripper are both initialized in autonomous init


		//		if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		//		{
		//			myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		//			autoLoopCounter++;
		//		}
		//		else
		//		{
		//			myRobot.Drive(0.0, 0.0); 	// stop robot
		//		}
#ifndef ECLIPSE
		driveLCD->UpdateLCD();
#endif
	} // end of AutonomousPeriodic

	void TeleopInit()
	{
		led_value = 1;
		compressor.Start();
#ifdef GRIPPER_START_CLOSED
		grip_is_open=false;
		gripper_control_solenoid.Set(GRIPPER_OPEN);
#else
		grip_is_open=true;
		gripper_control_solenoid.Set(GRIPPER_CLOSE);
#endif
		printf("gripper open");
#ifdef ECLIPSE
		SetDigitalOutputs4470(arduino_light0, arduino_light1, arduino_light2, led_value);
#endif
		printf("Compiled on %s\n",  COMPILED_TIME);

#ifdef ECLIPSE
		current_tote_height = GetDigitalInputValue4470(
				arduino_input_bit0,arduino_input_bit1,arduino_input_bit2,
				arduino_input_bit3,arduino_input_bit4);
#else
		current_tote_height = GetDigitalInputValue4470(
				arduino_input_bit0,arduino_input_bit1,arduino_input_bit2,
				arduino_input_bit3,arduino_input_bit4, 0);
#endif
		up_previously_pressed = false;
		down_previously_pressed = false;

#ifdef ECLIPSE
		power_board.ClearStickyFaults();
#endif
		//target_tote_height = current_tote_height;
		gripper_rotation_motor.Set(Relay::kOff);
	}

	void TeleopPeriodic()
	{
		//GAMEPAD RAW AXIS VALUES
		// Left X:0 Left Y: 1
		// Left Trigger: 2 Right Trigger: 3
		// Right X: 4 Right Y: 5
#ifdef ECLIPSE
		current_tote_height = GetDigitalInputValue4470(
				arduino_input_bit0,arduino_input_bit1,arduino_input_bit2,
				arduino_input_bit3,arduino_input_bit4);
#else
		current_tote_height = GetDigitalInputValue4470(
				arduino_input_bit0,arduino_input_bit1,arduino_input_bit2,
				arduino_input_bit3,arduino_input_bit4, 0);
#endif

#ifndef ECLIPSE
		DriverStationLCD *driveLCD = DriverStationLCD::GetInstance();
#endif

		ButtonPressedHandler();

		ArdWatchDogOutput();

		distance_using_tote_sensor = GetToteDistance();
		if (tote_sensor.GetValue() >= LOWEST_NUMBER_FOR_DISTANCE_ON_LINEAR_GRAPH &&
				tote_sensor.GetValue() <=HIGHEST_NUMBER_FOR_DISTANCE_ON_LINEAR_GRAPH)
		{
			sprintf( dashboard_output3,"distance: %d in",distance_using_tote_sensor);
			SmartDashboard::PutString("DB/String 3", dashboard_output3);

		}
		else if(tote_sensor.GetValue() < LOWEST_NUMBER_FOR_DISTANCE_ON_LINEAR_GRAPH)
		{
			SmartDashboard::PutString("DB/String 3", "Distance is <= 12 in");
		}
		else
		{
			SmartDashboard::PutString("DB/String 3","Distance is >= 120 in");
		}

		getCrateLiftTarget();

		sprintf(dashboard_output3,"use_tote_height = %d",use_tote_height);
		SmartDashboard::PutString("DB/String 8", dashboard_output3);

		sprintf(dashboard_output3,"crate mtr dir = %d",crate_motor_direction);
		SmartDashboard::PutString("DB/String 6", dashboard_output3);
		//printf("TTH %d\n",target_tote_height);


		if(button_box.GetRawButton(COMPRESSOR_START))
		{
			compressor.Start();
		}
		if(button_box.GetRawButton(COMPRESSOR_STOP))
		{
			compressor.Stop();
		}


		if(button_box.GetRawButton(INCREMENT_LED) && !increment_color_flag)
		{
			increment_color_flag = true;
			led_value++;
			//printf("LED output is %d\n", led_value);
		}
		else if(!button_box.GetRawButton(INCREMENT_LED) && increment_color_flag)
		{
			increment_color_flag = false;
		}
#ifdef ECLIPSE
		sprintf(dashboard_output1, "LEnc:%d, REnc:%d",left_encoder.GetRaw(),right_encoder.GetRaw());
		SmartDashboard::PutString("DB/String 5", dashboard_output1);
		//printf("The Joystick POV = %d\n", right_stick.GetPOV());
#endif
		checkTotePresence();
		//		sprintf( dashboard_output3,"tote_sensor: %d",tote_sensor.GetValue());
		//		SmartDashboard::PutString("DB/String 1", dashboard_output3);



		//printf("Top sensor = %s\n", top_limit_switch.Get()? "true":"false");
		//printf("Middle sensor = %s\n", middle_limit_switch.Get()? "true":"false");
		//printf("Bottom sensor = %s\n", bottom_limit_switch.Get()? "true":"false");

		ArduinoLightDisplay(led_value);
#ifdef ECLIPSE
		sprintf( dashboard_output3,"LED Value: %d",led_value);
		SmartDashboard::PutString("DB/String 9", dashboard_output3);
#endif
		//
		//#ifdef ECLIPSE
		//		SetDigitalOutputs4470(arduino_light0, arduino_light1, arduino_light2, led_value);
		//#endif

		//		bool reverseDir;
		//		reverseDir=switch_box.GetRawButton(REVERSE_DIR_DRIVE);

		// Switches the robot from arcade to tank
		drive_mode = switch_box.GetRawButton(ARCADE_SWITCH)?TANK_DRIVE:ARCADE_DRIVE;
		controller_drive_mode = switch_box.GetRawButton(GAME_PAD_SWITCH)?GAMEPAD:LEFT_STICK;

		sprintf(dashboard_output3,"Wheel inv: %d", reverse_direction);
		SmartDashboard::PutString("DB/String 4", dashboard_output3);

		if(drive_mode == ARCADE_DRIVE)
		{
			//			rightWheelSpeed = -1*rightWheelSpeed;


			if(controller_drive_mode == LEFT_STICK)
			{
				ArcadeDrive4470(right_stick.GetY() * reverse_direction, right_stick.GetX()* reverse_direction * -1, &left_wheel_speed, &right_wheel_speed, false);
#ifdef ECLIPSE

				sprintf( dashboard_output1,"RS Y:%.3f, X:%.3f",right_stick.GetY(),right_stick.GetX());
				SmartDashboard::PutString("DB/String 2", dashboard_output1);
				SmartDashboard::PutString("DB/String 7", "Arcade Drive with joystick");



#else
				driveLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Arcade Joystick");
				driveLCD->PrintfLine(DriverStationLCD::kUser_Line1, "y:%f x:%f", right_stick.GetY(),right_stick.GetX());

#endif
			}
			else
			{
				//ArcadeDrive4470(game_pad.GetY(), game_pad.GetX(), &left_wheel_speed, &right_wheel_speed, false);
				ArcadeDrive4470(game_pad.GetRawAxis(GAMEPAD_RIGHT_Y) * reverse_direction,
						game_pad.GetRawAxis(GAMEPAD_RIGHT_X) * reverse_direction * -1, &left_wheel_speed, &right_wheel_speed, false);
#ifdef ECLIPSE

				sprintf( dashboard_output1,"GP Y:%.3f, X:%.3f",
						game_pad.GetRawAxis(GAMEPAD_RIGHT_Y),game_pad.GetRawAxis(GAMEPAD_RIGHT_X));
				SmartDashboard::PutString("DB/String 2", dashboard_output1);
				SmartDashboard::PutString("DB/String 7", "Arcade with gamepad");
#else
				driveLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Arcade Gamepad");
				driveLCD->PrintfLine(DriverStationLCD::kUser_Line1, "y:%f x:%f", right_stick.GetY(),right_stick.GetX());

#endif
			}


		}
		else
		{
			if(controller_drive_mode == LEFT_STICK)
				// (and set a variable based on it, to be referenced later)
			{


				TankDrive4470(right_stick.GetY(),left_stick.GetY(),&left_wheel_speed,&right_wheel_speed);

#ifdef ECLIPSE

				sprintf( dashboard_output1,"RS Y: %.3f,LS Y: %.3f",right_stick.GetY(),left_stick.GetY());
				SmartDashboard::PutString("DB/String 2", dashboard_output1);
				SmartDashboard::PutString("DB/String 7", "Tank Drive with joystick");
#else
				driveLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Tank Drive with joystick");
#endif
			}
			else
			{

				TankDrive4470(game_pad.GetRawAxis(GAMEPAD_RIGHT_Y), game_pad.GetRawAxis(GAMEPAD_LEFT_Y),&left_wheel_speed, &right_wheel_speed);
#ifdef ECLIPSE

				sprintf( dashboard_output1,"GPL: %.3f GPR: %.3f ",game_pad.GetRawAxis(GAMEPAD_LEFT_Y),game_pad.GetRawAxis(GAMEPAD_RIGHT_Y));
				SmartDashboard::PutString("DB/String 2", dashboard_output1);
				SmartDashboard::PutString("DB/String 7", "Tank Drive with gamepad");
#else
				driveLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Tank Drive with gamepad");

#endif
			}
			//			if(reverseDir)
			//			{
			//				left_wheel_speed *=-1;
			//				right_wheel_speed *=-1;
			//#ifndef ECLIPSE
			//				driveLCD->PrintfLine(DriverStationLCD::kUser_Line3, "LWS %f RWS %f", left_wheel_speed, right_wheel_speed  );
			//#endif
			//
			////				*left_wheel_speed = -1**left_wheel_speed;
			////				*right_wheel_speed = -1**right_wheel_speed;
			//			}
			left_teleop_motor_power = left_wheel_speed;//when we have encoders fix this stuff
			right_teleop_motor_power = right_wheel_speed;
			//			leftMotor.Set(left_teleop_motor_power);
			//#ifndef ECLIPSE
			//				driveLCD->PrintfLine(DriverStationLCD::kUser_Line1, "LM %f", left_teleop_motor_power  );
			//
			//#endif
			//			rightMotor.Set(right_teleop_motor_power);
			//
			//#ifndef ECLIPSE
			//				driveLCD->PrintfLine(DriverStationLCD::kUser_Line2, "RM %f", right_teleop_motor_power  );
			//
			//#endif


			// ex: maintainPos(prev_right_wheel_speed, rightDriveSpeedIn,
			//					leftDriveSpeedOut, rightDriveSpeedOut,
			//					leftEncoderVal, rightEncoderVal)
			//			maintainPosition(prev_left_wheel_speed,prev_right_wheel_speed,
			//					left_wheel_speed,right_wheel_speed,left_encoder.GetRaw(),right_encoder.GetRaw());

			//			current_left_location = left_encoder.GetRaw();
			//			current_right_location = right_encoder.GetRaw();
			//
			//			if((left_wheel_speed == 0) && (right_wheel_speed == 0))
			//			{
			//				if((prev_left_wheel_speed == 0) && (prev_right_wheel_speed == 0))
			//				{
			//
			//					if(abs(current_left_location-stop_left_location) > 5 && abs(current_right_location-stop_left_location) > 5)
			//					{
			//
			//						left_teleop_motor_power=left_teleop_motor_power*(PROPORTIONAL_CONSTANT)*(abs(current_left_location-stop_left_location));
			//						right_teleop_motor_power=right_teleop_motor_power*(PROPORTIONAL_CONSTANT)*(abs(current_right_location-stop_right_location));
			//					}
			//
			//				}
			//				else
			//				{
			//					stop_left_location=current_left_location;
			//					stop_right_location=current_right_location;
			//				}
			//
			//			}
			//			prev_left_wheel_speed=left_wheel_speed;
			//			prev_right_wheel_speed=right_wheel_speed;
		}

		//		if(!has_tote && !switch_box.GetRawButton(USE_TOTE_PRESSENCE_SENSOR))
		//		if(!has_tote)
		//		{
		//			crate_lifting_motor.Set(Relay::kReverse);
		//		}
		//		right_wheel_speed = reverse_direction * invert_right * right_wheel_speed;
		//		left_wheel_speed = reverse_direction * invert_left * left_wheel_speed;
#ifdef ECLIPSE
		sprintf( dashboard_output1,"CTH: %d, TTH:%d",current_tote_height,target_tote_height);
		SmartDashboard::PutString("DB/String 0", dashboard_output1);

		//		sprintf( dashboard_output2,"target: %d",target_tote_height);
		//		SmartDashboard::PutString("DB/String 5", dashboard_output2);
#endif
		right_wheel_speed = invert_right * right_wheel_speed;
		left_wheel_speed = invert_left * left_wheel_speed;

		//printf( "RWS %f LWS %f\n",left_wheel_speed, right_wheel_speed);

		myRobot.SetLeftRightMotorOutputs(left_wheel_speed,right_wheel_speed);
#ifndef ECLIPSE
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line2, "RWS %f",right_wheel_speed);
		driveLCD->PrintfLine(DriverStationLCD::kUser_Line3, "LWS %f",left_wheel_speed);
		driveLCD->UpdateLCD();
#endif
		//printf("Right Stick Y Value  %f Right Stick X Value %f",  right_stick.GetY(),right_stick.GetX());
		//printf("RWS %f LWS %f",right_wheel_speed,left_wheel_speed);
		//		char axis_count[30];
		//		char db_string[30];
		//		char axis[30];
		//		sprintf( axis_count,"Axis Count: %d", game_pad.GetAxisCount());
		//		SmartDashboard::PutString("DB/String 6", axis_count);
		//		for(int i=0;i<game_pad.GetAxisCount();i++)
		//		{
		//			sprintf(db_string, "DB/String %d",i);
		//			sprintf( axis,"Axis %d: %f ",i,game_pad.GetRawAxis(i));
		//			SmartDashboard::PutString(db_string, axis);
		//			game_pad.GetRawAxis(i);
		//		}
		// need to test
		//printf("before the gripper rotation\n");


		if(grip_is_open) // set in teleop init no changes after init
		{
			gripper_control_solenoid.Set(GRIPPER_OPEN);

		}
		else
		{
			gripper_control_solenoid.Set(GRIPPER_CLOSE);
		}

		if(grip_extend)// set in the constuctor not changed in periodic
		{
			gripper_extension_solenoid.Set(DoubleSolenoid::kForward);
		}
		else
		{
			gripper_extension_solenoid.Set(DoubleSolenoid::kReverse);
		}


		//#ifndef DISABLE_TOTE_HEIGHT_USE
		//		if(use_tote_height)
		//		{
		//			//Tote height moves to target height and goes back if over shot
		//			if(target_tote_height > current_tote_height || !top_limit_switch.Get()) // called in teleop init initalized to current_tote_height printed in periodic and used in periodic
		//			{
		//				crate_lifting_motor.Set(CRATE_MOTOR_UP);
		//#ifdef ECLIPSE
		//				SmartDashboard::PutString("DB/String 8", "Going to Top");
		//#endif
		//			}
		//			if(target_tote_height < current_tote_height || !bottom_limit_switch.Get())
		//			{
		//				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
		//#ifdef ECLIPSE
		//				SmartDashboard::PutString("DB/String 8", "Going to Bottom");
		//#endif
		//			}
		//			if(target_tote_height == current_tote_height)
		//			{
		//				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
		//#ifdef ECLIPSE
		//				SmartDashboard::PutString("DB/String 8", "Going to Nowhere");
		//#endif
		//			}
		//		}
		//		else
		//		{
		//#endif
		//			if(right_stick.GetRawButton(CRATE_UP))
		//			{
		//				crate_lifting_motor.Set(CRATE_MOTOR_UP);
		//			}
		//			else if(right_stick.GetRawButton(CRATE_DOWN))
		//			{
		//				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
		//			}
		//			else
		//			{
		//				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
		//			}
		//#ifndef DISABLE_TOTE_HEIGHT_USE
		//		}
		//#endif


		switch(crate_motor_direction)
		{
		case FORWARD:
			crate_lifting_motor.Set(invert_lift ? CRATE_MOTOR_UP : CRATE_MOTOR_DOWN);
			break;
		case REVERSE:
			crate_lifting_motor.Set(invert_lift ? CRATE_MOTOR_DOWN : CRATE_MOTOR_UP);
			break;
		case OFF:
			crate_lifting_motor.Set(CRATE_MOTOR_OFF);
			break;
		default:
			crate_lifting_motor.Set(CRATE_MOTOR_OFF);
			break;
		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	//////////////////////////////////////////////////////////////
	// custom functions below                                   //
	//////////////////////////////////////////////////////////////

	void ButtonPressedHandler()
	{


		// reversing which side is front based on switch box
		//		switch_box.GetRawButton(REVERSE_DIR_DRIVE)?reverse_direction = -1:reverse_direction=1;


		reverse_direction = (!switch_box.GetRawButton(REVERSE_DIR_DRIVE)? -1:1);
		//		sprintf( dashboard_output1,"TFPUT: %f",left_encoder_starter_value);
		//		SmartDashboard::PutString("DB/String 4", dashboard_output1);

		if(right_stick.GetRawButton(GRIPPER_OUT))
		{
			grip_extend = true;
		}
		else if (right_stick.GetRawButton(GRIPPER_IN))
		{
			grip_extend = false;
		}


		grip_pressed=right_stick.GetRawButton(GRIP_TOGGLE);

		if(right_stick.GetRawButton(GRIP_OPEN))
		{
			grip_is_open = true;
		}
		else if(right_stick.GetRawButton(GRIP_CLOSE))
		{
			grip_is_open = false;
		}
		else if(!grip_prev_pressed && grip_pressed)
		{
			grip_is_open = !grip_is_open;

		}
		grip_prev_pressed = grip_pressed;
	}

	int GetToteDistance()
	{
		return (int)(SLOPE_TO_FIND_DISTANCE_APART*(int)tote_sensor.GetValue()+NUMBER_TO_ADD_NEEDED_TO_FIND_DISTANCE_APART);
	}

	void TankDrive4470(float left_axis_val, float right_axis_val, float *left_wheel_speed, float *right_wheel_speed)
	{
		if(reverse_direction == -1)
		{
			*left_wheel_speed = right_axis_val * reverse_direction;
			*right_wheel_speed = left_axis_val * reverse_direction;
		}
		else
		{
			*left_wheel_speed = left_axis_val * reverse_direction;
			*right_wheel_speed = right_axis_val * reverse_direction;
		}
	}


	void TankDriveJoystick4470(GenericHID &left_stick, GenericHID &right_stick, float *left_wheel_speed, float *right_wheel_speed)
	{
		if(reverse_direction == -1)
		{
			*left_wheel_speed = right_stick.GetY() * reverse_direction;
			*right_wheel_speed = left_stick.GetY() * reverse_direction;
		}
		else
		{
			*left_wheel_speed = left_stick.GetY() * reverse_direction;
			*right_wheel_speed = right_stick.GetY() * reverse_direction;
		}
	}

	void TankDriveGamePad4470(GenericHID &game_pad, double *left_wheel_speed, double *right_wheel_speed)
	{

		if(reverse_direction == -1)
		{
			*left_wheel_speed = game_pad.GetRawAxis(GAMEPAD_RIGHT_Y) * reverse_direction;
			*right_wheel_speed = game_pad.GetRawAxis(GAMEPAD_LEFT_Y) * reverse_direction;
		}
		else
		{
			*left_wheel_speed = game_pad.GetRawAxis(GAMEPAD_LEFT_Y) * reverse_direction;
			*right_wheel_speed = game_pad.GetRawAxis(GAMEPAD_RIGHT_Y) * reverse_direction;
		}
		printf("Left Wheel speed= %f\n",*left_wheel_speed);
		printf("Right Wheel  Speed= %f\n", *right_wheel_speed);
	}

	void  ArcadeDrive4470(float moveValue, float rotateValue, float *left_wheel_speed, float *right_wheel_speed, bool squaredInputs)
	{
		// local variables to hold the computed PWM values for the motors


		moveValue = Limit(moveValue);
		rotateValue = Limit(rotateValue);

		if (squaredInputs)
		{
			// square the inputs (while preserving the sign) to increase fine control while permitting full power
			if (moveValue >= 0.0)
			{
				moveValue = (moveValue * moveValue);
			}
			else
			{
				moveValue = -(moveValue * moveValue);
			}
			if (rotateValue >= 0.0)
			{
				rotateValue = (rotateValue * rotateValue);
			}
			else
			{
				rotateValue = -(rotateValue * rotateValue);
			}
		}

		if (moveValue > 0.0)
		{
			if (rotateValue > 0.0)
			{
				*left_wheel_speed = moveValue - rotateValue;

				*right_wheel_speed = Max(moveValue, rotateValue);

			}
			else
			{
				*left_wheel_speed = Max(moveValue, -rotateValue);
				*right_wheel_speed = moveValue + rotateValue;
			}
		}
		else
		{
			if (rotateValue > 0.0)
			{
				*left_wheel_speed = -Max(-moveValue, rotateValue);
				*right_wheel_speed = moveValue + rotateValue;
			}
			else
			{
				*left_wheel_speed = moveValue - rotateValue;
				*right_wheel_speed = -Max(-moveValue, -rotateValue);
			}
		}

		if(reverse_direction == -1)
		{
			float temp = *left_wheel_speed;
			*left_wheel_speed = *right_wheel_speed;
			*right_wheel_speed = temp;
		}

	}

	//copied from library code. is called in arcade to limit motor outputs from being !(-1<=output<=1)
	float Limit(float num)
	{
		if(num > 1.0)
		{
			return 1.0;
		}
		if (num < -1.0)
		{
			return -1.0;
		}
		return num;
	}
	float Max(float num1, float num2)
	{
		return (num1<num2)?num2:num1;
	}
	void SetDigitalOutputs4470(DigitalOutput &output0, DigitalOutput &output1, DigitalOutput &output2, int commandValue)
	{
		//gripper open && no tote = off
		//gripper open && yes tote && at bottom = yellow
		//gripper closed && no tote && at bottom = red
		//gripper closed && yes tote && at bottom = green
		//gripper closed && yes tote && at middle = orange
		//gripper closed && yes tote && at top = blue
		//gripper open && middle = pink
		//gripper open && at top = purple
		output0.Set(commandValue&1?1:0);
		output1.Set(commandValue&2?1:0);
		output2.Set(commandValue&4?1:0);
	}
	void checkTotePresence()
	{

#ifdef ECLIPSE
		sprintf( dashboard_output3,"tote_sensor: %d",tote_sensor.GetValue());
		SmartDashboard::PutString("DB/String 1", dashboard_output3);
		if(distance_using_tote_sensor < TOTE_SENSING_THRESHOLD)
		{
			has_tote = true;
		}
		else
		{
			has_tote = false;
		}
#else
		if(distance_using_tote_sensor<TOTE_SENSING_THRESHOLD)
		{
			has_tote=true;
		}
		else
		{
			has_tote=false;
		}

#endif
	}

	void pickingUpTotesInAutonomous()
	{

		printf("Auto state %d\n",auto_state);
#ifdef ECLIPSE
		sprintf( dashboard_output2,"AS =: %d",auto_state);
		SmartDashboard::PutString("DB/String 6", dashboard_output2);
		current_tote_height = GetDigitalInputValue4470(arduino_input_bit0,arduino_input_bit1,
				arduino_input_bit2,arduino_input_bit3,arduino_input_bit4);
		sprintf( dashboard_output3,"BLS: %d",bottom_limit_switch.Get()?1:0);
		SmartDashboard::PutString("DB/String 5", dashboard_output3);
		sprintf( dashboard_output1,"TLS: %d",top_limit_switch.Get()?1:0);
		SmartDashboard::PutString("DB/String 8", dashboard_output1);
		sprintf( dashboard_output2,"CTH: %d",current_tote_height);
		SmartDashboard::PutString("DB/String 4", dashboard_output2);

		sprintf( dashboard_output1,"Very long expression %d",!bottom_limit_switch.Get() ?1:0);
		//								 ||	((current_tote_height != 0) && (current_tote_height != 1) && (current_tote_height != 2) && (current_tote_height != 3))?1:0);
		SmartDashboard::PutString("DB/String 0", dashboard_output1);
#else
		//driveLCD->PrintfLine(DriverStationLCD::kUser_Line1, "auto_state %d",auto_state);
#endif
		//Ask if should be global
		bool reached_encoder_value;
		switch(auto_state)
		{
		case AUTO_PICK_UP_GOTO_BIN_HEIGHT:

			//				 printf("autonomous pick up state: case 0\n");
			if((current_tote_height < 1) || (current_tote_height == 63) || (timer.Get() - timer_starting_position < 2))
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at bottom/n");
				auto_state++;
				timer_starting_position=timer.Get();
			}
			//				if(!bottom_limit_switch.Get()
			//						&&((current_tote_height != 0) && (current_tote_height != 1) && (current_tote_height != 2) && (current_tote_height != 3)&&(current_tote_height!=63)))
			else if((current_tote_height > PICK_GARBAGE_CAN+1)
#ifdef USE_LIMIT_SWITCH
					&& !bottom_limit_switch.Get()
#endif
			)// && limit_switch_check)
			{
				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
				//				limit_switch_check = true;

			}
			else if((current_tote_height < PICK_GARBAGE_CAN-1)
#ifdef USE_LIMIT_SWITCH
					|| bottom_limit_switch.Get()/* && !limit_switch_check*/
#endif
			)
			{

				crate_lifting_motor.Set(CRATE_MOTOR_UP);
				//				limit_switch_check = false;

			}
			else
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at bottom/n");
				auto_state++;
				timer_starting_position=timer.Get();
				printf("timer %f",timer.Get());

			}
			break;
		case AUTO_PICK_UP_BIN_CASE: // case 1
			printf("autonomous pick up state: case 1\n");
			gripper_control_solenoid.Set(GRIPPER_CLOSE);
			grip_is_open =false;
			sprintf( dashboard_output2,"TFPUT: %f",timer_starting_position);
#ifdef ECLIPSE
			SmartDashboard::PutString("DB/String 0", dashboard_output2);
#endif
			if((timer.Get()-timer_starting_position) >= 0.02)
			{
				has_tote = true;
				auto_state++;
				printf("timer %f",timer.Get());
			}

			break;

			//						   if(timer.Get()-timer_starting_position>=2)
			//						   {


			//							   if(!top_limit_switch.Get() )
			//								   || current_tote_height != 20 )
		case AUTO_PICK_UP_GO_UP_AFTER_GRIP_RECYCLE_BIN:
			if(current_tote_height < GARBAGE_CAN_OVER_TOTE-1 &&  (timer.Get() - timer_starting_position < AUTONOMOUS_PICK_UP_STATE_STOP_LIFT_TIME)) //&& !top_limit_switch.Get())
			{
				crate_lifting_motor.Set(CRATE_MOTOR_UP);
			}
			else if(current_tote_height > GARBAGE_CAN_OVER_TOTE+1 && (timer.Get() - timer_starting_position < AUTONOMOUS_PICK_UP_STATE_STOP_LIFT_TIME))
			{
				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
			}
			else
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at top\n");
				auto_state++;
				printf("timer %f",timer.Get());

			}
			//						   }
			break;
		case AUTO_PICK_UP_DRIVE_TO_CONTAINER_CASE:
			printf("autonomous drive to container: case 2");
			//			left_encoder_starter_value=left_encoder.GetRaw();
			//			right_encoder_starter_value=right_encoder.GetRaw();

			//			sprintf( dashboard_output2,"TFPUT: %f",left_encoder_starter_value);
			//			SmartDashboard::PutString("DB/String 1", dashboard_output2);
			//			sprintf( dashboard_output1,"TFPUT: %f",left_encoder_starter_value);
			//			SmartDashboard::PutString("DB/String 4", dashboard_output1);
			printf("REV: %d",left_encoder_starter_value);
			printf("LEV: %d",right_encoder_starter_value);

			left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;// going same direction?
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();

			auto_state++;
			printf("timer %f",timer.Get());
			break;
		case AUTO_PICK_UP_STOP_DRIVE_TO_CONTAINER_CASE:
			printf("autonomous stop drive to container: case 3\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS)
			//					|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_PICK_UP_STATE_STOP_DRIVE_TO_CONTAINER_TIME))
			//			{
			//
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//				auto_state++;
			//				timer_starting_position = timer.Get();
			//
			//			}

			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_PICK_UP_STATE_STOP_DRIVE_TO_CONTAINER_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
				printf("timer %f",timer.Get());
			}

			break;
		case AUTO_PICK_UP_PLACE_RECYCLE_CASE:
			printf("autonomous place recyle on tote/grab tote: case 4\n");
			//					   if(timer.Get()-timer_starting_position<=2)
			//					   {
			//						   crate_lifting_motor.Set(Relay::kReverse);
			//					   }
			//					   else
			//					   {
			//						   crate_lifting_motor.Set(Relay::kOff);
			gripper_control_solenoid.Set(GRIPPER_CLOSE);
			grip_is_open =true;
			if((timer.Get()-timer_starting_position) >= 0.02)
			{
				has_tote = false;
				auto_state++;
				timer_starting_position = timer.Get();
				printf("timer %f",timer.Get());
			}
			//					   }
			break;
		case AUTO_PICK_UP_START_GRAB_TOTE_CASE: // rename to AUTO_PICK_UP_START_GRAB_TOTE_CASE
			//						   if(!bottom_limit_switch.Get() )
			//							   &&((current_tote_height != 0) && (current_tote_height != 1) && (current_tote_height != 2) && (current_tote_height  3)&&(current_tote_height!=63)))//Should it be top_limit_switch Answer: NO

			if(current_tote_height >BOTTOM_TOTE+2 && (timer.Get() - timer_starting_position < 2) )//&& !bottom_limit_switch.Get())
			{
				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
			}
			else
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at bottom\n");

				gripper_control_solenoid.Set(GRIPPER_OPEN);
				grip_is_open = false;

				auto_state++;
				timer_starting_position =timer.Get();
				printf("timer %f",timer.Get());
			}
			break;

		case AUTO_PICK_UP_FINISH_GRAB_TOTE_CASE: //  // rename to AUTO_PICK_UP_FINISH_GRAB_TOTE_CASE


			gripper_control_solenoid.Set(GRIPPER_OPEN);
			if((timer.Get()-timer_starting_position) >= 0.02)
			{
				has_tote = true;
				timer_starting_position = timer.Get();
				auto_state++;
				printf("timer %f",timer.Get());
			}
			break;
		case AUTO_PICK_UP_LIFT_TO_TOP_CASE:
			printf("In autonomous lift to top case");
			//								if(!top_limit_switch.Get())
			//									|| current_tote_height == 15 )
			if(current_tote_height < TOTE_OVER_SCORING_PLATFORM-1  && (timer.Get() - timer_starting_position < 2))// && !top_limit_switch.Get()) && why doesn't it working at 9
			{
				crate_lifting_motor.Set(CRATE_MOTOR_UP);
			}
			else if(current_tote_height > TOTE_OVER_SCORING_PLATFORM+1 && (timer.Get() - timer_starting_position < 2))
			{
				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
			}
			else
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at top\n");
				//									checkTotePresence();
				//									if(has_tote)
				//									{
				//									   auto_state++;
				//									}
				auto_state++;
				printf("timer %f",timer.Get());
			}


			break;

			//right now it is basically looping through continuously
		case AUTO_PICK_UP_TURN_LEFT_CASE:

			printf("autonomous turning left: case 5\n");
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			printf("REV: %d",left_encoder_starter_value);
			printf("LEV: %d",right_encoder_starter_value);
			left_autonomous_motor_power = -1*LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();


			auto_state++;
			printf("timer %f",timer.Get());
			break;

		case AUTO_PICK_UP_STOP_TURN_LEFT_CASE:
			printf("REV: %d",left_encoder_starter_value);
			printf("LEV: %d",right_encoder_starter_value);
			printf("autonomous stop turn left: case 6\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_TURN_NINETY_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_TURN_NINETY_CLICKS)
			//					|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_TURN_NINETY_CLICKS_TIME))
			//			{
			//
			//				timer_starting_position = timer.Get();
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//
			//				auto_state++;
			//
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_TURN_NINETY_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_TURN_NINETY_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_TURN_NINETY_CLICKS_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
				printf("timer %f",timer.Get());
			}

			break;
		case AUTO_PICK_UP_DRIVE_FORWARD_STATE:

			printf("autonomous drive forward: case 7\n");
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			printf("REV: %d",left_encoder_starter_value);
			printf("LEV: %d",right_encoder_starter_value);
			left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();

			auto_state++;
			printf("timer %f",timer.Get());
			break;

		case AUTO_PICK_UP_STOP_DRIVE_FORWARD_STATE:
			printf("autonomous stop drive forward: case 8\n");
			printf("REV: %d",left_encoder_starter_value);
			printf("LEV: %d",right_encoder_starter_value);

			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS)
			//					|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_TO_AUTOZONE_TIME))
			//			{
			//
			//				timer_starting_position = timer.Get();
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//				auto_state++;
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_FORWARD_TO_AUTOZONE_IN_PICKING_UP_TOTES_IN_AUTONOMOUS))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
				printf("timer %f",timer.Get());
			}
			break;
		default:
			printf("pickingUpTotesInAutonomous default");
			left_autonomous_motor_power = 0;
			right_autonomous_motor_power = 0;
			crate_lifting_motor.Set(CRATE_MOTOR_OFF);
			gripper_control_solenoid.Set(DoubleSolenoid::kOff);
			break;

		}
	}

	void extendingArmInAutonomous()
	{
		bool reached_encoder_value;
		switch(auto_state)
		{
		case 0:
			printf("extendingArmInAutonomous case 0\n");
			//begin driving backwards
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			timer.Start();
			left_autonomous_motor_power = -1*LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = -1*RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();
			printf("case 0 in auto_state");
			auto_state++;

			break;

		case 1:
			printf("extendingArmInAutonomous case 1\n");

			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_BACKWARD_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_BACKWARD_CLICKS)
			//					|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_BACKWARD_TIME))
			//			{
			//				timer_starting_position = timer.Get();
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//				auto_state++;
			//				printf("case 1 in auto_state");
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_BACKWARD_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_BACKWARD_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_BACKWARD_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
			}
			break;

		case 2:
			//extend arm
			printf("extendingArmInAutonomous case 2\n");
			printf("case 2 in auto_state");
			if(((timer.Get()-timer_starting_position)>=5))
			{
				auto_state++;
			}

			break;
		case 3:
			printf("extendingArmInAutonomous case 3\n");
			timer_starting_position = timer.Get();
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			auto_state++;
			printf("case 3 in auto_state");
			break;
		case 4:
			printf("extendingArmInAutonomous case 4\n");
			//			if(auto_with_encoder)
			//			{
			//				if((abs(-left_encoder_starter_value)>=AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS)
			//						|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_FORWARD_TIME))
			//				{
			//
			//					timer_starting_position = timer.Get();
			//					left_autonomous_motor_power = 0;
			//					right_autonomous_motor_power = 0;
			//
			//				}
			//			}
			//			else
			//			{
			//				if((abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_FORWARD_TIME))
			//				{
			//
			//					timer_starting_position = timer.Get();
			//					left_autonomous_motor_power = 0;
			//					right_autonomous_motor_power = 0;
			//
			//				}
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_FORWARD_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
			}
			printf("case 4 in auto_state");
			break;

		}
	}
	void drivingForwardInAutonomous()
	{
		// TODO: use switchbox switch to disable all references to the encoders,
		// in case the encoders are returning values that are not reasonable
		bool reached_encoder_value;
		switch(auto_state)
		{
		case 0:

			printf("drivingForwardInAutonomous state\n");
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			timer.Start();
			left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();
			printf("case 0 in auto_state");
			auto_state++;
			printf("timer %f",timer.Get());

			break;

		case 1:
			printf("drivingForwardInAutonomous default state\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS)
			//					|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_FORWARD_TIME))
			//			{
			//				timer_starting_position = timer.Get();
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//				printf("case 1 in auto_state");
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_DRIVE_FORWARD_STATE_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_FORWARD_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
				printf("timer %f",timer.Get());
			}
			break;
		case 2:
			printf("timer %f",timer.Get());
			left_autonomous_motor_power = 0;
			right_autonomous_motor_power = 0;
			break;
		default:

			// Note: should never get to default case
			printf("drivingForwardInAutonomous default state\n");


			timer_starting_position = timer.Get();
			left_autonomous_motor_power = 0;
			right_autonomous_motor_power = 0;

			break;
		}
	}

	void pushingTotesInAutonomous()
	{
		bool reached_encoder_value;
		switch(auto_state)
		{
		case AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_CASE:
			printf("pushingTotesInAutonomous case 0\n");
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			timer.Start();
			left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();
			auto_state++;
			break;
		case AUTONOMOUS_PUSH_TOTE_STOP_DRIVE_TO_CONTAINER_CASE:
			printf("pushingTotesInAutonomous case 1\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_STATE_CLICKS &&
			//					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_STATE_CLICKS)
			//					|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_TIME))
			//			{
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//				auto_state++;
			//
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_STATE_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_STATE_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
			}
			break;
		case AUTONOMOUS_LOWER_LIFT_CASE:
			printf("pushingTotesInAutonomous case 2\n");
			if(!bottom_limit_switch.Get())
			{
				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);

			}
			else
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at bottom\n");
				printf("timer %f",timer.Get());
				auto_state++;


			}
			break;
		case AUTONOMOUS_GRAB_CONTAINER_CASE:
			printf("pushingTotesInAutonomous case 3\n");
			gripper_control_solenoid.Set(GRIPPER_OPEN);
			printf("timer %f",timer.Get());
			auto_state++;
			break;
		case AUTONOMOUS_TURN_CASE:
			printf("pushingTotesInAutonomous case 4\n");
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			left_autonomous_motor_power = -1*AUTONOMOUS_PUSH_TOTE_TURN_CONSTANT*LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = -1*RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();
			printf("timer %f",timer.Get());
			auto_state++;

			break;
		case AUTONOMOUS_STOP_TURN_CASE:
			printf("pushingTotesInAutonomous case 5\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PUSH_TOTE_TURN_CLICKS*AUTONOMOUS_PUSH_TOTE_TURN_CONSTANT &&
			//					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PUSH_TOTE_TURN_CLICKS)
			//					|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_PUSH_TOTE_TURN_TIME))
			//			{
			//
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//
			//				auto_state++;
			//
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PUSH_TOTE_TURN_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PUSH_TOTE_TURN_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_PUSH_TOTE_TURN_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				printf("timer %f",timer.Get());
				auto_state++;
			}
			break;
		case AUTONOMOUS_DRIVE_FORWARD_CASE:
			printf("pushingTotesInAutonomous case 6\n");
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();

			left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();
			printf("timer %f",timer.Get());
			auto_state++;
			break;
		case AUTONOMOUS_STOP_DRIVE_FORWARD_CASE:
			printf("pushingTotesInAutonomous case 7\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PUSH_TOTE_DRIVE_FORWARD_STATE_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PUSH_TOTE_DRIVE_FORWARD_STATE_CLICKS)
			//					|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_PUSH_TOTE_DRIVE_FORWARD_TIME))
			//			{
			//
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//
			//				auto_state++;
			//
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PUSH_TOTE_DRIVE_FORWARD_STATE_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PUSH_TOTE_DRIVE_FORWARD_STATE_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_PUSH_TOTE_DRIVE_FORWARD_TIME))
			{

				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
				// TODO: go to stopped case
			}
			break;
			// TODO: stay stopped case
		default:
			left_autonomous_motor_power = 0;
			right_autonomous_motor_power = 0;
			gripper_control_solenoid.Set(DoubleSolenoid::kOff);
			crate_lifting_motor.Set(CRATE_MOTOR_OFF);
			break;
		}



	}

	void pickBinAndPushTotes()
	{
#ifdef ECLIPSE
		sprintf( dashboard_output2,"AS =: %d",auto_state);
		SmartDashboard::PutString("DB/String 6", dashboard_output2);
		current_tote_height = GetDigitalInputValue4470(arduino_input_bit0,arduino_input_bit1,
				arduino_input_bit2,arduino_input_bit3,arduino_input_bit4);
		sprintf( dashboard_output3,"BLS: %d",bottom_limit_switch.Get()?1:0);
		SmartDashboard::PutString("DB/String 5", dashboard_output3);
		sprintf( dashboard_output1,"TLS: %d",top_limit_switch.Get()?1:0);
		SmartDashboard::PutString("DB/String 8", dashboard_output1);
		sprintf( dashboard_output2,"CTH: %d",current_tote_height);
		SmartDashboard::PutString("DB/String 4", dashboard_output2);

		sprintf( dashboard_output1,"Very long expression %d",!bottom_limit_switch.Get() ?1:0);
		//								 ||	((current_tote_height != 0) && (current_tote_height != 1) && (current_tote_height != 2) && (current_tote_height != 3))?1:0);
		SmartDashboard::PutString("DB/String 0", dashboard_output1);
#else

		//driveLCD->PrintfLine(DriverStationLCD::kUser_Line4, "auto_state %d",auto_state);
#endif

		// TODO: only reference encoders if switch on switchbox is set correctly
		bool reached_encoder_value;
		switch(auto_state)
		{
		case AUTO_SETUP:
			timer_starting_position = timer.Get();
			auto_state++;
			printf("timer %f",timer.Get());
			printf("Going to AUTO_GET_TO_BIN_HEIGHT: case %d\n", AUTO_GET_TO_BIN_HEIGHT);
			break;
		case AUTO_GET_TO_BIN_HEIGHT:

			//			printf("got to the 0rth case in pickBinAndPushTotes\n");
			if((current_tote_height < 1) || (current_tote_height == 63) || (timer.Get() - timer_starting_position < 2))
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at bottom/n");
				printf("timer %f",timer.Get());
				auto_state++;
				printf("Going to AUTO_GRAB_BIN: case %d\n", AUTO_GRAB_BIN);
				timer_starting_position=timer.Get();
			}
			else if((current_tote_height > PICK_GARBAGE_CAN+1)
#ifdef USE_LIMIT_SWITCH
					&& !bottom_limit_switch.Get()
#endif
			)
				// && limit_switch_check)
			{
				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
				//				limit_switch_check = true;

			}
			else if((current_tote_height < PICK_GARBAGE_CAN-1)
#ifdef USE_LIMIT_SWITCH
					|| bottom_limit_switch.Get()/* && !limit_switch_check*/
#endif
			)
			{

				crate_lifting_motor.Set(CRATE_MOTOR_UP);
				//				limit_switch_check = false;

			}
			else
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at bottom/n");
				auto_state++;
				timer_starting_position=timer.Get();
				printf("timer %f",timer.Get());
				printf("Going to AUTO_GRAB_BIN: case %d\n", AUTO_GRAB_BIN);
			}
			break;
		case AUTO_GRAB_BIN:


			gripper_control_solenoid.Set(GRIPPER_CLOSE);
			grip_is_open = false;
			sprintf( dashboard_output2,"TFPUT: %f",timer_starting_position);
#ifdef ECLIPSE
			SmartDashboard::PutString("DB/String 0", dashboard_output2);
#endif
			if(timer.Get()-timer_starting_position >= 0.02)
			{
				has_tote = true;
				timer_starting_position = timer.Get();
				auto_state++;
				printf("timer %f",timer.Get());
				printf("Going to AUTO_GO_UP_AFTER_GRAB_BIN: case %d\n", AUTO_GO_UP_AFTER_GRAB_BIN);
			}
			break;

		case AUTO_GO_UP_AFTER_GRAB_BIN:
			//			printf("got to the 2nd case in pickBinAndPushTotes\n");
			if((current_tote_height < GARBAGE_CAN_OVER_TOTE-1)  && (timer.Get() - timer_starting_position < AUTONOMOUS_PICK_UP_STATE_STOP_LIFT_TIME))//&& !top_limit_switch.Get())
			{
				crate_lifting_motor.Set(CRATE_MOTOR_UP);
			}
			else if((current_tote_height > GARBAGE_CAN_OVER_TOTE+1) && (timer.Get() - timer_starting_position < AUTONOMOUS_PICK_UP_STATE_STOP_LIFT_TIME))
			{
				crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
			}
			else
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at top\n");
				auto_state++;
				printf("timer %f",timer.Get());
				printf("Going to AUTO_DRIVING_FORWARD: case %d\n", AUTO_DRIVING_FORWARD);
			}
			break;
		case AUTO_DRIVING_FORWARD:

			//printf("autonomous drive to container: case 2");

			left_encoder_starter_value=left_encoder_value;
			right_encoder_starter_value=right_encoder_value;
			//			sprintf( dashboard_output2,"TFPUT: %f",left_encoder_starter_value);
			//			SmartDashboard::PutString("DB/String 1", dashboard_output2);
			//			sprintf( dashboard_output1,"TFPUT: %f",left_encoder_starter_value);
			//			SmartDashboard::PutString("DB/String 4", dashboard_output1);
			printf("REV: %d",left_encoder_starter_value);
			printf("LEV: %d",right_encoder_starter_value);

			left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;// going same direction?
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();

			auto_state++;
			printf("timer %f",timer.Get());
			printf("Going to AUTO_STOP_DRIVING_FORWARD: case %d\n", AUTO_STOP_DRIVING_FORWARD);
			break;
		case AUTO_STOP_DRIVING_FORWARD:
			//			printf("got to the 4nd case in pickBinAndPushTotes\n");
			//printf("autonomous stop drive to container: case 3\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS
			//					&& abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS)
			//					|| (timer.Get()-timer_starting_position>=AUTONOMOUS_PICK_UP_STATE_STOP_DRIVE_TO_CONTAINER_TIME))
			//			{
			//
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//				auto_state++;
			//				timer_starting_position = timer.Get();
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_PICK_UP_STATE_STOP_DRIVE_TO_CONTAINER_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
				printf("timer %f",timer.Get());
				printf("Going to AUTO_TURNING_LEFT: case %d\n", AUTO_TURNING_LEFT);
			}
			break;
		case AUTO_TURNING_LEFT:
			//			printf("got to the 5rd case in pickBinAndPushTotes");
			//printf("autonomous turning left: case 5\n");
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			printf("REV: %d",left_encoder_starter_value);
			printf("LEV: %d",right_encoder_starter_value);
			left_autonomous_motor_power = -1*LEFT_AUTONOMOUS_DRIVE_POWER;
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();
			auto_state++;
			printf("timer %f",timer.Get());
			printf("Going to AUTO_STOP_TURNING_LEFT: case %d\n", AUTO_STOP_TURNING_LEFT);
			break;
		case AUTO_STOP_TURNING_LEFT:
			//			printf("got to the 6rth case in pickBinAndPushTotes");
			printf("REV: %d",left_encoder_value);
			printf("LEV: %d",right_encoder_value);
			//			printf("autonomous stop turn left: case 6\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_TURN_NINETY_CLICKS
			//					&& abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_TURN_NINETY_CLICKS)
			//					|| (timer.Get()-timer_starting_position>=AUTONOMOUS_TURN_NINETY_CLICKS_TIME))
			//			{
			//
			//				timer_starting_position = timer.Get();
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//
			//				auto_state++;
			//
			//			}
			//Turn Left may be subject to change
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_TURN_NINETY_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_TURN_NINETY_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_TURN_NINETY_CLICKS_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
				printf("timer %f",timer.Get());
				printf("Going to AUTO_DRIVING_TO_AUTO_ZONE: case %d\n", AUTO_DRIVING_TO_AUTO_ZONE);
			}
			break;
		case AUTO_DRIVING_TO_AUTO_ZONE:
			//			printf("got to the 7th case in pickBinAndPushTotes");
			//printf("autonomous drive to container: case 2");
			left_encoder_starter_value=left_encoder.GetRaw();
			right_encoder_starter_value=right_encoder.GetRaw();
			//			sprintf( dashboard_output2,"TFPUT: %f",left_encoder_starter_value);
			//			SmartDashboard::PutString("DB/String 1", dashboard_output2);
			//			sprintf( dashboard_output1,"TFPUT: %f",left_encoder_starter_value);
			//			SmartDashboard::PutString("DB/String 4", dashboard_output1);
			printf("REV: %d",left_encoder_starter_value);
			printf("LEV: %d",right_encoder_starter_value);

			left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;// going same direction?
			right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
			timer_starting_position = timer.Get();
			auto_state++;
			printf("timer %f",timer.Get());
			printf("Going to AUTO_STOP_DRIVING_TO_AUTO_ZONE: case %d\n", AUTO_STOP_DRIVING_TO_AUTO_ZONE);
			break;

		case AUTO_STOP_DRIVING_TO_AUTO_ZONE:

			//printf("autonomous stop drive to container: case 3\n");
			//			if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS
			//					&& abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS)
			//					|| (timer.Get()-timer_starting_position>=AUTONOMOUS_DRIVE_TO_AUTOZONE_TIME))
			//			{
			//
			//				left_autonomous_motor_power = 0;
			//				right_autonomous_motor_power = 0;
			//				auto_state++;
			//				timer_starting_position = timer.Get();
			//			}
			reached_encoder_value=(abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_BIN_STATE_MOVE_TO_AUTO_ZONE_CLICKS &&
					abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_BIN_STATE_MOVE_TO_AUTO_ZONE_CLICKS);
			if(((auto_with_encoder)&&reached_encoder_value)
					|| ((timer.Get()-timer_starting_position)>=AUTONOMOUS_PICK_BIN_STATE_MOVE_TO_AUTOZONE_TIME))
			{
				timer_starting_position = timer.Get();
				left_autonomous_motor_power = 0;
				right_autonomous_motor_power = 0;
				auto_state++;
				printf("timer %f",timer.Get());
				printf("Going to AUTO_STOP: case %d\n", AUTO_STOP);

			}
			break;
		case AUTO_STOP:
			left_autonomous_motor_power = 0;
			right_autonomous_motor_power = 0;

			break;
		default:
			left_autonomous_motor_power = 0;
			right_autonomous_motor_power = 0;
			break;
		}
	}

	void getCrateLiftTarget()
	{
#ifndef DISABLE_TOTE_HEIGHT_USE
		use_tote_height = (switch_box.GetRawButton(HEIGHT_LIFT_MOVE) && ((current_tote_height != 0)
				&& (current_tote_height != 63)));
#else
		use_tote_height = false;
#endif

		// old code of target limit switch moved to oldCode.h
		if(use_tote_height)
		{

			//////////////////////////////////////////////////////////////////////
			// determine direction of lift mechanism based on buttonbox inputs  //
			//////////////////////////////////////////////////////////////////////
			if(button_box.GetRawButton(BOTTOM_TOTE_BUTTON))
			{
				target_tote_height = BOTTOM_TOTE;
				target_button_pressed = true;
				got_to_tote_target = false;
				prev_crate_lifter_dir = current_tote_height - target_tote_height > 0?REVERSE:OFF;
			}
			if(button_box.GetRawButton(TOTE_OVER_SCORING_PLATFORM_BUTTON))
			{
				target_tote_height = TOTE_OVER_SCORING_PLATFORM;
				target_button_pressed = true;
				got_to_tote_target = false;
				if(current_tote_height - target_tote_height > 0)
				{
					prev_crate_lifter_dir = REVERSE;
				}
				else if(current_tote_height - target_tote_height < 0)
				{
					prev_crate_lifter_dir = FORWARD;
				}
				else
				{
					prev_crate_lifter_dir = OFF;
				}
			}
			if(button_box.GetRawButton(PICK_GARBAGE_CAN_BUTTON))
			{
				target_tote_height = PICK_GARBAGE_CAN;
				target_button_pressed = true;
				got_to_tote_target = false;
				if(current_tote_height - target_tote_height > 0)
				{
					prev_crate_lifter_dir = REVERSE;
				}
				else if(current_tote_height - target_tote_height < 0)
				{
					prev_crate_lifter_dir = FORWARD;
				}
				else
				{
					prev_crate_lifter_dir = OFF;
				}
			}
			if(button_box.GetRawButton(TOTE_OVER_TOTE_BUTTON))
			{
				target_tote_height = TOTE_OVER_TOTE;
				target_button_pressed = true;
				got_to_tote_target = false;
				if(current_tote_height - target_tote_height > 0)
				{
					prev_crate_lifter_dir = REVERSE;
				}
				else if(current_tote_height - target_tote_height < 0)
				{
					prev_crate_lifter_dir = FORWARD;
				}
				else
				{
					prev_crate_lifter_dir = OFF;
				}
			}
			if(button_box.GetRawButton(TOTE_OVER_STEP_BUTTON))
			{
				target_tote_height = TOTE_OVER_STEP;
				target_button_pressed = true;
				got_to_tote_target = false;
				if(current_tote_height - target_tote_height > 0)
				{
					prev_crate_lifter_dir = REVERSE;
				}
				else if(current_tote_height - target_tote_height < 0)
				{
					prev_crate_lifter_dir = FORWARD;
				}
				else
				{
					prev_crate_lifter_dir = OFF;
				}
			}
			if(button_box.GetRawButton(GARBAGE_CAN_OVER_TOTE_BUTTON))
			{
				target_tote_height = GARBAGE_CAN_OVER_TOTE;
				target_button_pressed = true;
				got_to_tote_target = false;
				if(current_tote_height - target_tote_height > 0)
				{
					prev_crate_lifter_dir = REVERSE;
				}
				else if(current_tote_height - target_tote_height < 0)
				{
					prev_crate_lifter_dir = FORWARD;
				}
				else
				{
					prev_crate_lifter_dir = OFF;
				}
			}


		}
		else
		{
			target_tote_height = -1;
			got_to_tote_target = false;
		}
		//////////////////////////////////////////////////////////////////////
		// determine direction of lift mechanism based on joystick inputs   //
		//////////////////////////////////////////////////////////////////////

		// setting lift motor targets
		if(right_stick.GetRawButton(CRATE_UP))
		{
#ifdef USE_HEIGHT_LIMIT_SWITCHES
			if (top_limit_switch.Get())
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at top\n");
			}
			else
			{
#endif
				//					crate_lifting_motor.Set(CRATE_MOTOR_UP);
				crate_motor_direction = FORWARD;
				target_button_pressed = false;
				target_tote_height = -1;
				got_to_tote_target = false;
#ifdef USE_HEIGHT_LIMIT_SWITCHES
			}
#endif



		}
		else if(right_stick.GetRawButton(CRATE_DOWN))
		{

#ifdef USE_HEIGHT_LIMIT_SWITCHES
			if (bottom_limit_switch.Get())
			{
				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
				printf("crate lifter at bottom\n");
			}
			else
			{
#endif
				//					crate_lifting_motor.Set(CRATE_MOTOR_DOWN);
				crate_motor_direction = REVERSE;
				target_button_pressed = false;
				target_tote_height = -1;
				got_to_tote_target = false;
#ifdef USE_HEIGHT_LIMIT_SWITCHES
			}

#endif
		}

		else
		{
			//				crate_lifting_motor.Set(CRATE_MOTOR_OFF);
			crate_motor_direction = OFF;
		}

		if((target_button_pressed) && (target_tote_height >= 1))
		{
			if(current_tote_height > target_tote_height && prev_crate_lifter_dir == REVERSE)
			{
				crate_motor_direction = REVERSE;
			}
			else if(current_tote_height < target_tote_height && prev_crate_lifter_dir == FORWARD)
			{
				crate_motor_direction = FORWARD;
			}
			else
			{
				prev_crate_lifter_dir = OFF;
				crate_motor_direction = OFF;
			}
		}
		//		got_to_tote_target = got_to_tote_target || (current_tote_height == target_tote_height);
		//		if(got_to_tote_target)
		//		{
		//			crate_motor_direction = OFF;
		//		}
	}

	void rotateGripperMotors()
	{
		if (right_stick.GetRawButton(GRIPPER_FORWARD_ROTATION) && right_stick.GetRawButton(GRIPPER_BACKWARD_ROTATION) )
		{
			//printf("before off\n");
			//			SmartDashboard::PutString("DB/String 6", "Gripper Rot Off");
			gripper_rotation_motor.Set(Relay::kOff);
		}
		else if(right_stick.GetRawButton(GRIPPER_FORWARD_ROTATION))
		{
			//printf("before forward\n");
			//			SmartDashboard::PutString("DB/String 6", "Gripper Rot Forward");
			gripper_rotation_motor.Set(Relay::kForward);
		}
		else if (right_stick.GetRawButton(GRIPPER_BACKWARD_ROTATION))
		{
			//printf("before reverse\n");
			//			SmartDashboard::PutString("DB/String 6", "Gripper Rot Backwards");
			gripper_rotation_motor.Set(Relay::kReverse);

		}
		else
		{
			//printf("before else\n");
			//			SmartDashboard::PutString("DB/String 6", "Gripper Rot Off");
			gripper_rotation_motor.Set(Relay::kOff);
		}
	}

	void maintainPosition(float& leftDriveSpeedIn, float& rightDriveSpeedIn,
			float& leftDriveSpeedOut, float& rightDriveSpeedOut,
			float leftEncoderVal, float rightEncoderVal)
	{
		current_left_location = left_encoder.GetRaw();
		current_right_location = right_encoder.GetRaw();

		if((left_wheel_speed == 0) && (right_wheel_speed == 0))
		{
			if((prev_left_wheel_speed == 0) && (prev_right_wheel_speed == 0))
			{

				if(abs(current_left_location-stop_left_location) > LEFT_DRIVE_ENCODER_BACKLASH && abs(current_right_location-stop_right_location) > RIGHT_DRIVE_ENCODER_BACKLASH)
				{

					left_teleop_motor_power=left_teleop_motor_power*(PROPORTIONAL_CONSTANT)*(abs(current_left_location-stop_left_location));
					right_teleop_motor_power=right_teleop_motor_power*(PROPORTIONAL_CONSTANT)*(abs(current_right_location-stop_right_location));
				}

			}
			else
			{
				stop_left_location=current_left_location;
				stop_right_location=current_right_location;
			}

		}
		prev_left_wheel_speed=left_wheel_speed;
		prev_right_wheel_speed=right_wheel_speed;
	}

	void ArduinoLightDisplay(uint8_t &led_value)
	{
		if(grip_is_open)
		{
			if(!has_tote)
			{
				led_value = 0;
				//				printf("\n no tote, grip open\n");
			}
			else if(current_tote_height<=BOTTOM_TOTE+2)
			{
				led_value = 1;
				//				printf("grip open, has tote, at bottom\n");
			}
			else if(current_tote_height>=TOTE_OVER_TOTE)
			{
				led_value = 4;
				//				printf("grip closed, has tote, at top\n");
			}
			else
			{
				led_value = 6;
			}
		}
		else // gripper closed
		{
			if(!has_tote)
			{
				led_value = 2;
				//					printf("grip closed, no tote\n");
			}
			else if(current_tote_height<=BOTTOM_TOTE+2)
			{
				led_value = 3;
				//				printf("grip closed, has tote, at bottom\n");
			}
			else if (current_tote_height>=TOTE_OVER_TOTE)
			{
				led_value = 5;
				//				printf("grip closed, has tote, at top\n");
			}
			else
			{
				led_value = 7;

			}
		}

#ifdef ECLIPSE
		SetDigitalOutputs4470(arduino_light0, arduino_light1, arduino_light2,led_value);
#endif
	}
	//		if(grip_is_open && !has_tote)
	//		{
	//
	//			led_value = 0;
	//			printf("\n no tote, grip open\n");
	//		}
	//		elif(grip_is_open && has_tote && current_tote_height<BOTTOM_TOTE+2)
	//		{
	//
	//			led_value = 1;
	//			printf("grip open, has tote, at bottom\n");
	//		}
	//		elif(!grip_is_open && !has_tote && current_tote_height<BOTTOM_TOTE+2)
	//		{
	//
	//			led_value = 2;
	//			printf("grip closed, no tote\n, at bottom");
	//		}
	//		elif(!grip_is_open && has_tote && current_tote_height<BOTTOM_TOTE+2)
	//		{
	//
	//			led_value = 3;
	//			printf("grip closed, has tote, at bottom\n");
	//		}
	//
	//		elif(!grip_is_open && has_tote && current_tote_height>=TOTE_OVER_TOTE)
	//		{
	//
	//			led_value = 5;
	//			printf("grip closed, has tote, at top\n");
	//		}
	//		else
	//		{
	//			printf("error: Unknown state\n");
	//			led_value = 6;
	//		}


	//this code is old,has been revised above
	//	void pickingUpTotesInAutonomousOld()
	//	{
	//	   switch(auto_state)
	//	   {
	//	   	   case AUTONOMOUS_PUSH_TOTE_DRIVE_TO_CONTAINER_CASE:
	//	   		   	   	    printf("autonomous drive to container: case 0");
	//						left_encoder_starter_value=left_encoder.GetRaw();
	//						right_encoder_starter_value=right_encoder.GetRaw();
	//						timer.Start();
	//						left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
	//						right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
	//						timer_starting_position = timer.Get();
	//						auto_state++;
	//						break;
	//	   	   case AUTONOMOUS_PUSH_TOTE_STOP_DRIVE_TO_CONTAINER_CASE:
	//					printf("autonomous stop drive to container: case 1\n");
	//					if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_DRIVE_TO_CONTAINER_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS)
	//							|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_TO_AUTOZONE_TIME))
	//					{
	//
	//						timer_starting_position = timer.Get();
	//						left_autonomous_motor_power = 0;
	//						right_autonomous_motor_power = 0;
	//
	//						auto_state++;
	//
	//					}
	//		   	   	break;
	//		   case AUTONOMOUS_PICK_UP_CASE:
	//			   		   printf("autonomous pick up state: case 2\n");
	//					   gripper_control_solenoid.Set(GRIPPER_OPEN);
	//					   if(!top_limit_switch.Get())
	//					   {
	//						crate_lifting_motor.Set(Relay::kForward);
	//					   }
	//					   else
	//						{
	//							crate_lifting_motor.Set(Relay::kOff);
	//							printf("crate lifter at top\n");
	//							auto_state++;
	//
	//						}
	//						break;
	//		   case AUTONOMOUS_TURN_LEFT_CASE:
	//
	//						printf("autonomous turning left: case 3\n");
	//						left_encoder_starter_value=left_encoder.GetRaw();
	//						right_encoder_starter_value=right_encoder.GetRaw();
	//
	//						left_autonomous_motor_power = -1*LEFT_AUTONOMOUS_DRIVE_POWER;
	//						right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
	//						timer_starting_position = timer.Get();
	//
	//						auto_state++;
	//						break;
	//
	//		   case AUTONOMOUS_STOP_TURN_LEFT_CASE:
	//
	//						printf("autonomous stop turn left: case 4\n");
	//						if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_TURN_NINETY_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_TURN_NINETY_CLICKS)
	//								|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_TURN_NINETY_CLICKS_TIME))
	//						{
	//
	//							timer_starting_position = timer.Get();
	//							left_autonomous_motor_power = 0;
	//							right_autonomous_motor_power = 0;
	//
	//							auto_state++;
	//
	//						}
	//						break;
	//		   case PICK_UP_AUTONOMOUS_DRIVE_FORWARD_STATE:
	//
	//						printf("autonomous drive forward: case 5\n");
	//						left_encoder_starter_value=left_encoder.GetRaw();
	//						right_encoder_starter_value=right_encoder.GetRaw();
	//
	//						left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
	//						right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
	//						timer_starting_position = timer.Get();
	//
	//						auto_state++;
	//						break;
	//
	//		   case PICK_UP_AUTONOMOUS_STOP_DRIVE_FORWARD_STATE:
	//						printf("autonomous stop drive forward: case 6\n");
	//						if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS)
	//								|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_TO_AUTOZONE_TIME))
	//						{
	//
	//							timer_starting_position = timer.Get();
	//							left_autonomous_motor_power = 0;
	//							right_autonomous_motor_power = 0;
	//
	//							auto_state++;
	//
	//						}
	//						break;
	//		   case AUTONOMOUS_TURN_RIGHT_CASE:
	//						printf("autonomous turn right: case 7\n");
	//						left_encoder_starter_value=left_encoder.GetRaw();
	//						right_encoder_starter_value=right_encoder.GetRaw();
	//
	//						left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
	//						right_autonomous_motor_power = -1*RIGHT_AUTONOMOUS_DRIVE_POWER;
	//						timer_starting_position = timer.Get();
	//
	//						auto_state++;
	//						break;
	//
	//		   case AUTONOMOUS_STOP_TURN_RIGHT_CASE:
	//						printf("autonomous stop turn right: case 8\n");
	//						if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_TURN_NINETY_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_TURN_NINETY_CLICKS)
	//								|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_TURN_NINETY_CLICKS_TIME))
	//						{
	//
	//							timer_starting_position = timer.Get();
	//							left_autonomous_motor_power = 0;
	//							right_autonomous_motor_power = 0;
	//
	//							auto_state++;
	//
	//						}
	//						break;
	//		   case AUTONOMOUS_DRIVE_HORIZONTAL_STATE:
	//
	//						printf("autonomous drive horizontal: case 9\n");
	//						left_encoder_starter_value=left_encoder.GetRaw();
	//						right_encoder_starter_value=right_encoder.GetRaw();
	//
	//						left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
	//						right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
	//						timer_starting_position = timer.Get();
	//
	//						auto_state++;
	//						break;
	//
	//		   case AUTONOMOUS_STOP_DRIVE_HORIZONTAL_STATE:
	//						printf("autonomous stop drive horizontal: case 10\n");
	//						if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_MOVE_HORIZONTAL_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_HORIZONTAL_CLICKS)
	//								|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_PICK_UP_STATE_MOVE_HORIZONTAL_TIME))
	//						{
	//
	//							timer_starting_position = timer.Get();
	//							left_autonomous_motor_power = 0;
	//							right_autonomous_motor_power = 0;
	//
	//							auto_state++;
	//							printf("case 1 in auto_state");
	//						}
	//						break;
	//		   case SECOND_AUTONOMOUS_TURN_RIGHT_CASE:
	//						printf("autonomous turn right: case 11\n");
	//						left_encoder_starter_value=left_encoder.GetRaw();
	//						right_encoder_starter_value=right_encoder.GetRaw();
	//
	//						left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
	//						right_autonomous_motor_power = -1*RIGHT_AUTONOMOUS_DRIVE_POWER;
	//						timer_starting_position = timer.Get();
	//
	//						auto_state++;
	//						break;
	//
	//		   case SECOND_AUTONOMOUS_STOP_TURN_RIGHT_CASE:
	//						printf("autonomous stop turn right: case 12\n");
	//						if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_TURN_NINETY_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_TURN_NINETY_CLICKS)
	//								|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_TURN_NINETY_CLICKS_TIME))
	//						{
	//
	//							timer_starting_position = timer.Get();
	//							left_autonomous_motor_power = 0;
	//							right_autonomous_motor_power = 0;
	//
	//							auto_state++;
	//
	//						}
	//						break;
	//		   case SECOND_AUTONOMOUS_DRIVE_FORWARD_STATE:
	//
	//						printf("autonomous drive forward: case 13\n");
	//						left_encoder_starter_value=left_encoder.GetRaw();
	//						right_encoder_starter_value=right_encoder.GetRaw();
	//
	//						left_autonomous_motor_power = LEFT_AUTONOMOUS_DRIVE_POWER;
	//						right_autonomous_motor_power = RIGHT_AUTONOMOUS_DRIVE_POWER;
	//						timer_starting_position = timer.Get();
	//
	//						auto_state++;
	//						break;
	//
	//		   case SECOND_PICK_UP_AUTONOMOUS_STOP_DRIVE_FORWARD_STATE:
	//						printf("autonomous stop drive forward: case 14\n");
	//						if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS)
	//								|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_TO_AUTOZONE_TIME))
	//						{
	//
	//							timer_starting_position = timer.Get();
	//							left_autonomous_motor_power = 0;
	//							right_autonomous_motor_power = 0;
	//
	//							auto_state++;
	//
	//						}
	//						break;
	//		   case SECOND_PLACE_RECYCLE_AND_GRAB_STATE:
	//				   printf("autonomous place recyle on tote/grab tote: case 15\n");
	//				   gripper_control_solenoid.Set(GRIPPER_CLOSE);
	//				   crate_lifting_motor.Set(Relay::kReverse);
	//					if (bottom_limit_switch.Get())
	//					{
	//						crate_lifting_motor.Set(Relay::kOff);
	//						printf("crate lifter at bottom\n");
	//					}
	//					gripper_control_solenoid.Set(GRIPPER_OPEN);
	//				   crate_lifting_motor.Set(Relay::kForward);
	//					if (top_limit_switch.Get())
	//					{
	//						crate_lifting_motor.Set(Relay::kOff);
	//						printf("crate lifter at top\n");
	//					}
	//				   checkTotePresence();
	//				   if(has_tote)
	//				   {
	//					   auto_state++;
	//				   }
	//				  break;
	//		   case AUTONOMOUS_DRIVE_BACKWARD_STATE:
	//
	//						printf("autonomous drive backward: case 16\n");
	//						left_encoder_starter_value=left_encoder.GetRaw();
	//						right_encoder_starter_value=right_encoder.GetRaw();
	//
	//						left_autonomous_motor_power = -1*LEFT_AUTONOMOUS_DRIVE_POWER;
	//						right_autonomous_motor_power = -1*RIGHT_AUTONOMOUS_DRIVE_POWER;
	//						timer_starting_position = timer.Get();
	//
	//						auto_state++;
	//						break;
	//
	//		   case AUTONOMOUS_STOP_DRIVE_BACKWARD_STATE:
	//						printf("autonomous stop drive backward: case 17\n");
	//						if((abs(left_encoder_value-left_encoder_starter_value)>=AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS && abs(right_encoder_value-right_encoder_starter_value)>= AUTONOMOUS_PICK_UP_STATE_MOVE_TO_AUTO_ZONE_CLICKS)
	//								|| (abs(timer.Get()-timer_starting_position)>=AUTONOMOUS_DRIVE_TO_AUTOZONE_TIME))
	//						{
	//							timer_starting_position = timer.Get();
	//							left_autonomous_motor_power = 0;
	//							right_autonomous_motor_power = 0;
	//							auto_state++;
	//						}
	//						break;
	//	   	   }
	//	}
#ifdef ECLIPSE
	int GetDigitalInputValue4470(DigitalInput &input0, DigitalInput &input1, DigitalInput &input2, DigitalInput &input3,
			DigitalInput &input4)
#else
	int GetDigitalInputValue4470(DigitalInput &input0, DigitalInput &input1, DigitalInput &input2, DigitalInput &input3,
			DigitalInput &input4, int input_value5)
#endif
	{
		int input_value = 0;

		if(input0.Get())
		{
			input_value += 1;
		}

		if(input1.Get())
		{
			input_value += 2;
		}

		if(input2.Get())
		{
			input_value += 4;
		}

		if(input3.Get())
		{
			input_value += 8;
		}

		if(input4.Get())
		{
			input_value += 16;
		}

		return input_value;
	}
	void ArdWatchDogOutput()
	{
		if(ard_output_cycle_counter >= ARD_CYCLE_LIMIT)
		{
			ard_output_cycle_counter = 0;
			ard_live_signal_value = !ard_live_signal_value;
		}
		else
		{
			ard_output_cycle_counter++;
		}
		arduino_signal_bit.Set(ard_live_signal_value?1:0);
	}
};
START_ROBOT_CLASS(Robot);
