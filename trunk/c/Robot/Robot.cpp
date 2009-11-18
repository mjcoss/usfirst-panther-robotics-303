#include <iostream.h>
#include <math.h>

#include "AxisCamera.h" 
#include "BaeUtilities.h"
#include "FrcError.h"
#include "TrackAPI.h" 
#include "Jaguar.h"
#include "Victor.h"
#include "Encoder.h"
#include "Accelerometer.h"
#include "Gyro.h"
#include "Joystick.h"
#include "Servo.h"
#include "Victor.h"
#include "DriverStationLCD.h"
#include "DriverStation.h"
#include "WPILib.h"

#include "Target.h"
#include "Robot.h"

robotStatus_t robotInfo;

float
conformValue(float val, float min, float max)
{
	if(val > max)
		return max;
	if(val < min)
		return min;
	return val;
}

class Robbie : public IterativeRobot
{
	// Declare a variable to use to access the driver station object
	DriverStation *ds;
	DriverStationLCD *dsLCD;

	// Joysticks stuff
	Joystick *sticks[NUM_JOYSTICKS]; 
	bool buttonState[NUM_JOYSTICKS][(NUM_JOYSTICK_BUTTONS+1)];

	// Local variables to count the number of periodic loops performed
	UINT32 loopCount[NUM_OPERATING_MODES];

	Jaguar *driveMotors[NUM_DRIVE_MOTORS];
	
	Victor *shooterMotors[NUM_SHOOTER_MOTORS];
	Victor *pickupMotors[NUM_PICKUP_MOTORS];
	Victor *gatekeeper;
	
	Encoder *freeWheel[NUM_FREE_WHEEL_ENCODERS];
	Encoder *driveWheel[NUM_DRIVE_WHEEL_ENCODERS];

	Accelerometer *axis[NUM_AXISES];
	
	Gyro *gyros[NUM_AXISES];

#if USE_CAMERA
	Servo *panServo;
	Servo *tiltServo;
#endif
	
public:
	// constructor
	Robbie(void)
	{
		SetDebugFlag(DEBUG_SCREEN_ONLY);

		dprintf(LOG_DEBUG, "Robbie Constructor Started");

		ds               = DriverStation::GetInstance();
		dsLCD            = DriverStationLCD::GetInstance();
		sticks[0]        = new Joystick(1);
		sticks[1]        = new Joystick(2);
		gyros[X_AXIS]    = new Gyro(1);
		gyros[Y_AXIS]    = new Gyro(2);
		axis[X_AXIS]     = new Accelerometer(3);
		axis[Y_AXIS]     = new Accelerometer(4);
		axis[Z_AXIS]     = new Accelerometer(5);
		driveMotors[0]   = new Jaguar(1);
		driveMotors[1]   = new Jaguar(2);
		shooterMotors[0] = new Victor(3);
		shooterMotors[1] = new Victor(5);
		gatekeeper       = new Victor(6);
		// hack associate pickupMotors to single pwm
		pickupMotors[0]  = new Victor(7);
		pickupMotors[1]  = pickupMotors[0];

		freeWheel[ENCODER_LEFT]  = new Encoder(2, 3, true,  CounterBase::k4X);
		freeWheel[ENCODER_RIGHT] = new Encoder(4, 5, false, CounterBase::k4X);
		driveWheel[ENCODER_LEFT] = new Encoder(6, 7, true,  CounterBase::k4X);
		driveWheel[ENCODER_RIGHT]= new Encoder(8, 9, false, CounterBase::k4X);
#if USE_CAMERA
		panServo         = new Servo(9);
		tiltServo        = new Servo(10);
#endif
		GetWatchdog().SetExpiration(1.0);
		GetWatchdog().SetEnabled(true);		

		dprintf(LOG_DEBUG, "Robbie Constructor Completed");
	}
	
	void 
	RobotInit(void)
	{
		UINT32 i;
		UINT8 buttonNum;
		
		dprintf(LOG_DEBUG, "Robbie Init Started");	
		robotInfo.pktNum    = 0;
		robotInfo.pktPerSec = 0;

		for(i = 0; i < NUM_JOYSTICKS; i++) {
			for(buttonNum = 1; buttonNum < NUM_JOYSTICK_BUTTONS; buttonNum++)
				buttonState[i][buttonNum] = false;
		}

		gyros[X_AXIS]->SetSensitivity(MY_GYRO_SENSITIVITY);
		gyros[X_AXIS]->Reset();

		gyros[Y_AXIS]->SetSensitivity(MY_GYRO_SENSITIVITY);
		gyros[Y_AXIS]->Reset();

		for(i = 0; i<NUM_FREE_WHEEL_ENCODERS; i++ ) {
			freeWheel[i]->SetMaxPeriod(FREE_WHEEL_MAX_PERIOD);
			freeWheel[i]->SetMinRate(FREE_WHEEL_MIN_RATE);
			freeWheel[i]->SetDistancePerPulse(FREE_WHEEL_DISTANCE_PER_PULSE);
			freeWheel[i]->Reset();
		}
		
		for (i = 0; i<NUM_DRIVE_WHEEL_ENCODERS; i++) {
			driveWheel[i]->SetMaxPeriod(DRIVE_WHEEL_MAX_PERIOD);
			driveWheel[i]->SetMinRate(DRIVE_WHEEL_MIN_RATE);
			driveWheel[i]->SetDistancePerPulse(DRIVE_WHEEL_DISTANCE_PER_PULSE);
			driveWheel[i]->Reset();
		}
		
		for(i = 0; i < NUM_AXISES; i++) {
			axis[i]->SetZero(MY_ACCELEROMETER_ZERO);
			axis[i]->SetSensitivity(MY_ACCELEROMETER_SENSITIVITY);
		}
		
		for(i = 0; i < NUM_OPERATING_MODES; i++)
			loopCount[i] = 0;

		switch ((ds->GetDigitalIn(1) << 1) | ds->GetDigitalIn(2)) {
			case 3: robotInfo.StartingPosition = MIDDLE; break;
			case 2: robotInfo.StartingPosition = NEAR;   break;
			case 1: robotInfo.StartingPosition = FAR;    break;
			default: 
				dprintf(LOG_ERROR, "Unknown starting position: check inputs");
				robotInfo.StartingPosition = FAR;
				break;
		}
#ifdef TRACTION_CONTROL
		robotInfo.traction = MIN_CONTROL;
#endif
		
#if USE_CAMERA 
		/* start the CameraTask	 */
		if (StartCameraTask(FRAMERATE, 0, k160x120, ROT_180) == -1) {
			dprintf(LOG_ERROR, "Failed to spawn camera task; exiting. Error code %s", 
					GetVisionErrorText(GetLastVisionError()));
		}
		
		// start up camera tracking task
		initCambot(panServo, tiltServo);
#endif
		/* allow writing to vxWorks target */
		Priv_SetWriteFileAllowed(1);   		

		dprintf(LOG_DEBUG, "RobotInit() completed");
	}
	
	void
	Drive(float speed, float curve, Jaguar *left, Jaguar *right)
	{
		static const float sensitivity = 0.1;
		float leftSpeed, rightSpeed;
		
		if (curve < 0) {
			float value = log(-curve);
			float ratio = (value - sensitivity)/(value + sensitivity);
			if (ratio == 0) 
				ratio = .0000000001;
			leftSpeed = speed / ratio;
			rightSpeed = speed;
		}
		else if (curve > 0) {
			float value = log(curve);
			float ratio = (value - sensitivity)/(value + sensitivity);
			if (ratio == 0) 
				ratio = .0000000001;
			leftSpeed = speed;
			rightSpeed = speed / ratio;
		}
		else
		{
			leftSpeed  = speed;
			rightSpeed = speed;
		}
		left->Set(leftSpeed);
		right->Set(rightSpeed);
		return;
	}
	
	void
	UpdateDriverStationStatus()
	{
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Shooter Speed: %.3f", 
				(ds->GetAnalogIn(3) * 0.0048875855)/ 5.0);
		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " Pickup Speed: %.3f", 
				(ds->GetAnalogIn(4) * 0.0048875855)/ 5.0);
		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "  Shoot ball?: %s", 
				(ds->GetDigitalIn(3)) ? "no " : "yes");
		switch(robotInfo.StartingPosition) {
			case NEAR:
				dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Starting Pos: Near   ");
				break;
			case MIDDLE:
				dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Starting Pos: Middle ");
				break;
			case FAR:
				dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Starting Pos: Far    ");
				break;
			default:
				dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Starting Pos: Unknown");
				break;
		}
		dsLCD->UpdateLCD();
	}

	void
	initSensors()
	{
		int i;
		
		gyros[X_AXIS]->Reset();
		gyros[Y_AXIS]->Reset();

		for(i=0; i<NUM_FREE_WHEEL_ENCODERS; i++) {
			freeWheel[i]->Reset();
			freeWheel[i]->Start();
		}
		
		for(i=0; i<NUM_DRIVE_WHEEL_ENCODERS; i++) {
			driveWheel[i]->Reset();
			driveWheel[i]->Start();
		}
		
		for(i=0; i<NUM_AXISES; i++) {
			robotInfo.acceleration[i]  = 0.0;
			robotInfo.offsets[i]       = axis[i]->GetAcceleration();
		}
		return;
	}
	
	bool
	enableGate(float currentSpeed, unsigned int *waitTime)
	{
		// Define rules for enaging feeder
		if(fabs(currentSpeed) < 0.5)
			return false;
		if(--(*waitTime))
			return false;
		return true;
	}
	
	void
	collectSensorData()
	{
		float heading;
		
		for(int i = 0; i < NUM_AXISES; i++)
			robotInfo.acceleration[i] = axis[i]->GetAcceleration() - robotInfo.offsets[i];
		
		for(int i = 0; i < NUM_FREE_WHEEL_ENCODERS; i++)
			robotInfo.free_wheel_encoders[i] = freeWheel[i]->Get();
		for(int i = 0; i < NUM_DRIVE_WHEEL_ENCODERS; i++)
			robotInfo.drive_wheel_encoders[i] = driveWheel[i]->Get();
		
		heading = gyros[X_AXIS]->GetAngle();
		heading -= 360.0*floor(heading/360.0);
		robotInfo.x_heading = (double)heading;
		
		heading = gyros[Y_AXIS]->GetAngle();
		heading -= 360.0*floor(heading/360.0);
		robotInfo.y_heading = (double)heading;
		return;
	}
	
	void 
	DisabledInit(void) 
	{
		loopCount[DISABLED] = 0;			// Reset the loop counter for disabled mode

		// reset sensors
		initSensors();

		dprintf(LOG_DEBUG, "DisabledInit() completed");
	}
	
	void 
	AutonomousInit(void)
	{
		loopCount[AUTO] = 0;				// Reset the loop counter for autonomous mode

		// reset sensors
		initSensors();
		
		dprintf(LOG_DEBUG, "AutonomousInit() completed");
	}
	
	void 
	TeleopInit(void) 
	{
		loopCount[TELEOP]   = 0;			// Reset the loop counter for teleop mode
		robotInfo.pktPerSec = 0;			// Reset the number of dsPackets in current second

		// reset sensors
		initSensors();
		
		dprintf(LOG_DEBUG, "TeleopInit() completed");
	}

	void 
	DisabledPeriodic(void)
	{
		// feed the user watchdog at every period when disabled
		GetWatchdog().Feed();
		
		// increment the number of disabled periodic loops completed
		loopCount[DISABLED]++;
		
		if(loopCount[DISABLED] % 2) {
			// collectSensorData();
		}
		
		// while disabled, printout the duration of current disabled mode in seconds
		if ((loopCount[DISABLED] % (UINT32)GetLoopsPerSec()) == 0) {
			dprintf(LOG_DEBUG, "Disabled seconds: %d", 
					(loopCount[DISABLED] / (UINT32)GetLoopsPerSec()));
			UpdateDriverStationStatus();
		}
	}
	
	void 
	AutonomousPeriodic(void) 
	{
		// static const float Kp = 0.3;
		static UINT32 travelTime;
		UINT32 nSeconds;
		
		// feed the user watchdog at every period when in autonomous
		GetWatchdog().Feed();				
		travelTime = (robotInfo.StartingPosition == MIDDLE) ? 2 : 4;
		nSeconds = loopCount[AUTO]++ / (UINT32)GetLoopsPerSec();
		
		// note first time thur we're  blind, but we should be ok...
		if((robotInfo.free_wheel_encoders[0] < 750) && (nSeconds < travelTime)) {
			// Drive(0.25, -Kp * robotInfo.x_heading, driveMotors[0], driveMotors[1]);
			driveMotors[0]->Set(-0.45);
			driveMotors[1]->Set(0.45);
		}
		// spin for rest of autonomus period
		if(nSeconds > 4) {
			driveMotors[0]->Set(0.5);
			driveMotors[1]->Set(0.5);
		}
		
		if(loopCount[AUTO] % 2) {
			// collectSensorData();
		}
		
		if((loopCount[AUTO] % (UINT32)GetLoopsPerSec()) == 0) {
			dprintf(LOG_DEBUG, "Autonomus %d seoonds", nSeconds);
			dprintf(LOG_DEBUG, "free wheel encoder %g %g", 
					robotInfo.free_wheel_encoders[0], robotInfo.free_wheel_encoders[1]);
			UpdateDriverStationStatus();
		}
	}

	void 
	TeleopPeriodic(void) 
	{
		static UINT32 waitTime;
		float currentJoystickLeft;
		float currentJoystickRight;

#ifdef TRACTION_CONTROL
		static double microLastTime;
		double microCurrentTime;
		double microTimeElapsed;

		float currentMotorSpeedLeft;
		float driveEncoderRateLeft;
		float followEncoderRateLeft;
		float encoderDeltaLeft;
		float motorDeltaLeft;
		float proactLeft, reactLeft;
		
		float currentMotorSpeedRight;
		float driveEncoderRateRight;
		float followEncoderRateRight;
		float encoderDeltaRight;
		float motorDeltaRight;
		float proactRight, reactRight;
		
		float leftThrottle;
		float rightThrottle;
		float proactiveSensitivity;
		float reactiveSensitivity;
#endif //TRACTION_CONTROL
		GetWatchdog().Feed();
		
		currentJoystickLeft   = sticks[LEFTSIDE]->GetY();
		currentJoystickRight  = sticks[RIGHTSIDE]->GetY();	

		// increment the number of teleop periodic loops completed
		loopCount[TELEOP]++;
		
#ifdef TRACTION_CONTROL
		// compute traction control setting from presets based on button push vrs throttle
		// when validated, remove call to GetThrottle()
		switch (robotInfo.traction) {
			case NO_CONTROL:
				leftThrottle  = NO_PROACTIVE; rightThrottle = NO_REACTIVE;
				break;
			case MIN_CONTROL:
				leftThrottle = MIN_PROACTIVE; rightThrottle = MIN_REACTIVE;
				break;
			case MAX_CONTROL:
				leftThrottle = MAX_PROACTIVE; rightThrottle = MAX_REACTIVE;
				break;
			default:
				leftThrottle = NO_PROACTIVE; rightThrottle = NO_REACTIVE;
				break;
		};
		// leftThrottle  = sticks[LEFTSIDE]->GetThrottle();
		// rightThrottle = sticks[RIGHTSIDE]->GetThrottle();
		
		currentMotorSpeedLeft = driveMotors[0]->Get();
		driveEncoderRateLeft  = robotInfo.drive_wheel_encoders[0];
		followEncoderRateLeft = robotInfo.free_wheel_encoders[0];
		
		currentMotorSpeedRight = -driveMotors[1]->Get();
		driveEncoderRateRight  = robotInfo.drive_wheel_encoders[1];
		followEncoderRateRight = robotInfo.free_wheel_encoders[1];

		//Proactive sensitivity is how much the traction control will dampen the input from the joysticks
		proactiveSensitivity = (1.0 + leftThrottle)/2.0; //CHANGE THIS!
		proactiveSensitivity *= PROACTIVE_FACTOR;
		
		//Reactive sensitivity controls how major a change the code can make to the motor values to compensate for slipping or sliding
		reactiveSensitivity = (1.0 - rightThrottle)/2.0; //CHANGE THIS!
		reactiveSensitivity *= REACTIVE_FACTOR;
		
		//This is the timing section of the loop
		//We need to know the time elapsed in order to find the maximum motor change
		microCurrentTime = GetTime(); //Updates the current time
		microTimeElapsed = ElapsedTime(microLastTime); //Calculate time elapsed since last update
		microLastTime = microCurrentTime; //Prepares timer for next loop

		//We take the value on the joystick, put a deadzone (JOYSTICK_TRIM) on it, and compare this to the current motor values to see if we need to make any changes
		//The proactive part of the traction control is implemented here
		//The proactive sensitivity in this case is basically equivalent to the coefficient of friction
		//Maximum acceleration is g multiplied by the coefficient of friction

		//If the throttle is all the way down, put the change right through
		if(leftThrottle > 1.0 - THROTTLE_DEADZONE)
		    proactLeft = currentJoystickLeft - currentMotorSpeedLeft;

		//If the joystick wants the motors to move more forwards/less backwards, set motor delta to a positive value
		else if(currentJoystickLeft - JOYSTICK_TRIM > currentMotorSpeedLeft)
		    proactLeft =  proactiveSensitivity * G_FEET * microTimeElapsed;

		//If the joystick wants the motors to move less forwards/more backwards, set motor delta to a negative value
		else if(currentJoystickLeft + JOYSTICK_TRIM < currentMotorSpeedLeft)
		    proactLeft = -proactiveSensitivity * G_FEET * microTimeElapsed;

		//If the joystick is within the deadzone of the current motor value, make no change
		else
		    proactLeft = 0.0;
		
		//Do the same thing for the right side
		if(leftThrottle > (1.0 - THROTTLE_DEADZONE))
		    proactRight = currentJoystickRight - currentMotorSpeedRight;
		else if(currentJoystickRight - JOYSTICK_TRIM > currentMotorSpeedRight)
		    proactRight = proactiveSensitivity * G_FEET * microTimeElapsed;
		else if(currentJoystickRight + JOYSTICK_TRIM < currentMotorSpeedRight)
		    proactRight = -proactiveSensitivity * G_FEET * microTimeElapsed;
		else
		    proactRight = 0.0;

		//If the follower encoder rate is positive, the robot is moving forwards; likewise, a negative value means the robot is moving backwards
		//If the drive encoder is positive, the robot is trying to drive forwards; likewise, a negative value means the robot is trying to move backwards
		//If both the follower and drive encoder values are positive we have two options:
		// + the follower encoder value is greater than the drive encoder value (encoderDelta is positive)
		//   - this would mean that the robot is sliding forwards
		//   - we want the motors to try and move more quickly forwards
		//   - followerEncoderRate minus driveEncoderRate is positive
		// + the follower encoder value is less than the drive encoder value (encoderDelta is negative)
		//   - this would mean that the robot is slipping forwards
		//   - we want the motors to try and move less quickly forwards
		//If the follower encoder value is negative and the drive encoder value is positive:
		// + motorDelta will always be negative
		//   - we are trying to reverse the direction of the robot, and the wheels are slipping
		//   - we want the motors to move less quickly forwards
		//If the follower encoder value is positive and the drive encoder is negative:
		// + motorDelta will always be positive
		//   - we are trying to reverse the direction of the robot, and the wheels are slipping
		//   - we want the motors to move less quickly backwards
		//If both the follower and drive encoder values are negative, again we have two options:
		// + the follower encoder value is more negative than the drive encoder value (encoderDelta is negative)
		//   - this would mean that the robot is sliding backwards
		//   - we want to motors to try and move more quickly backwards
		// + the follower encoder value is less negativge than the drive encoder value (encoderDelta is positive)
		//   - this would mean that the robot is slipping backwards
		//   - we want the robot to move less quickly backwards
		encoderDeltaLeft  = followEncoderRateLeft  - driveEncoderRateLeft;
		encoderDeltaRight = followEncoderRateRight - driveEncoderRateRight;

		//As we want the robot to be more sensitive to smaller differences in encoder rates, and more reactive to larger changes,
		//we cube the encoderDelta to remap its values, then multiply by the sentivity we calculates before
		if(rightThrottle < (1.0 - THROTTLE_DEADZONE)) {
		   reactLeft  = (reactiveSensitivity * encoderDeltaLeft  * microTimeElapsed);
		   reactRight = (reactiveSensitivity * encoderDeltaRight * microTimeElapsed);
		}
		else
			reactLeft = reactRight = 0.0;
		
		motorDeltaLeft  = proactLeft  + reactLeft;
		motorDeltaRight = proactRight + reactRight;
		
		//Finally, we set the motor value to the current speed plus the change
		driveMotors[0]->Set(currentMotorSpeedLeft  + motorDeltaLeft);
		driveMotors[1]->Set(-(currentMotorSpeedRight + motorDeltaRight));

#else
#ifdef SQUARE_INPUTS
		if(currentJoystickLeft > 0)
			currentJoystickLeft = currentJoystickLeft * currentJoystickLeft;
		else
			currentJoystickLeft = -(currentJoystickLeft * currentJoystickLeft);
		if(currentJoystickRight > 0)
			currentJoystickRight = currentJoystickRight * currentJoystickRight;
		else
			currentJoystickRight = -(currentJoystickRight * currentJoystickRight);
#endif // SQUARE_INPUTS
		driveMotors[0]->Set(currentJoystickLeft); 
		driveMotors[1]->Set(-currentJoystickRight);
#endif // !TRACTION_CONTROL
		
		if ((loopCount[TELEOP] % 2) == 0){
			// 100 Hz processing here
			float shooterSpeed = (ds->GetAnalogIn(3) * 0.0048875855)/ 5.0;
			float currentSpeed = shooterMotors[0]->Get();
			float pickupSpeed  = (ds->GetAnalogIn(4) * 0.0048875855)/ 5.0;
			float keeperSpeed;
			bool runGate;
			
			shooterSpeed = conformValue(shooterSpeed, 0.0, 1.0);
			// delay running gatekeeper motor till the shooter has spun up
			if((currentSpeed == 0.0) && (shooterSpeed > 0.0))
					waitTime = 4;
			runGate = true;
			if(waitTime)
				runGate = enableGate(currentSpeed, &waitTime);

			shooterMotors[0]->Set(-shooterSpeed);
			shooterMotors[1]->Set(-shooterSpeed);

			pickupSpeed = conformValue(pickupSpeed,   0.0, 1.0);
			pickupMotors[0]->Set(pickupSpeed);
			pickupMotors[1]->Set(pickupSpeed);
		
			// let monentary switch govern ball drop mechanism
			// keeperSpeed = (ds->GetDigitalIn(3)) ? 0.0 : 1.0;
			keeperSpeed = 0.0;
			if((sticks[LEFTSIDE]->GetTrigger() && sticks[RIGHTSIDE]->GetTrigger()))
				keeperSpeed = -0.7;
			else if ((sticks[RIGHTSIDE]->GetTrigger()) || (sticks[LEFTSIDE]->GetTrigger()))
				keeperSpeed = 0.7;
			else
				keeperSpeed = 0.0;
			//if(runGate)
				gatekeeper->Set(keeperSpeed);
			//else
			//	gatekeeper->Set(0.0);
			// collectSensorData();
		}
		
		if ((loopCount[TELEOP] % 4) == 0){
			// 50 Hz processing here
		}

		if (ds->GetPacketNumber() != robotInfo.pktNum) {		 
			robotInfo.pktNum = ds->GetPacketNumber();
			robotInfo.pktPerSec++;
		}
		
		if ((loopCount[TELEOP] % (UINT32)GetLoopsPerSec()) == 0) {
			dprintf(LOG_DEBUG, "         Teleop seconds: %d", (loopCount[TELEOP] / (UINT32)GetLoopsPerSec()));
			dprintf(LOG_DEBUG, "             DS Packets: %u", robotInfo.pktPerSec);
#ifdef TRACTION_CONTROL
			/*
			dprintf(LOG_DEBUG, "   traction adjust left: %3.2f, right: %3.2f",
					motorDeltaLeft, motorDeltaRight);
			dprintf(LOG_DEBUG, "   throttle values left: %3.2f, right %3.2f",
					leftThrottle, rightThrottle);
			dprintf(LOG_DEBUG, "  proactiveSensitivity: %3.2f",
					proactiveSensitivity);
			dprintf(LOG_DEBUG, "   reactiveSensitivity: %3.2f",
					reactiveSensitivity);
			dprintf(LOG_DEBUG, "  Current Motor left: %3.2f, Joystick left: %3.2f",
					currentMotorSpeedLeft,  currentJoystickLeft);
			dprintf(LOG_DEBUG, "  Current Motor right: %3.2f, Joystick right: %3.2f",
					currentMotorSpeedRight, currentJoystickRight);
			dprintf(LOG_DEBUG, "        Right proact: %3.2f, react: %3.2f",
					proactRight, reactRight);
			dprintf(LOG_DEBUG, "        Left  proact: %3.2f, react: %3.2f",
					proactLeft,  reactLeft);
			*/
			switch(robotInfo.traction) {
				case NO_CONTROL:  dprintf(LOG_DEBUG, "BUTTON BASED TRACTION SETTING: NONE"); break;
				case MIN_CONTROL: dprintf(LOG_DEBUG, "BUTTON BASED TRACTION SETTING:  MIN"); break;
				case MAX_CONTROL: dprintf(LOG_DEBUG, "BUTTON BASED TRACTION SETTING:  MAX"); break;
				default: dprintf(LOG_DEBUG, "INVALID TRACTION SETTINGS"); break;
			}
#endif
			dprintf(LOG_DEBUG, "    encoder free wheels: %g, %g", 
					robotInfo.free_wheel_encoders[0], robotInfo.free_wheel_encoders[1]);
			dprintf(LOG_DEBUG, "    left free wheel rate: %g drive wheel rate %g",
					freeWheel[0]->GetRate(), driveWheel[0]->GetRate());
			dprintf(LOG_DEBUG, "    right"
					" free wheel rate: %g drive wheel rate %g",
					freeWheel[1]->GetRate(), driveWheel[1]->GetRate());
			dprintf(LOG_DEBUG, "    encoder drive wheels: %g, %g", 
								robotInfo.drive_wheel_encoders[0], robotInfo.drive_wheel_encoders[1]);
			dprintf(LOG_DEBUG, "Current victor6 info %g ", 
					pickupMotors[0]->Get());
			dprintf(LOG_DEBUG, "Current victor7 info %g ",
					gatekeeper->Get());

			// send info to DS
			UpdateDriverStationStatus();
			robotInfo.pktPerSec = 0;
		}
	}

	void 
	DisabledContinuous(void)
	{
		// grab sensor data; this should be fine -- need to test
		collectSensorData();
	}

	void 
	AutonomousContinuous(void)
	{
		// grab sensor data; this should be fine -- need to test
		collectSensorData();
	}

	void 
	TeleopContinuous(void)
	{
		// grab sensor data; this should be fine -- need to test
		collectSensorData();
#ifdef TRACTION_CONTROL
		if(sticks[LEFTSIDE]->GetRawButton(3))
			robotInfo.traction = NO_CONTROL;
		if(sticks[LEFTSIDE]->GetRawButton(4))
			robotInfo.traction = MIN_CONTROL;
		if(sticks[LEFTSIDE]->GetRawButton(5))
			robotInfo.traction = MAX_CONTROL;
#endif
	}
};

START_ROBOT_CLASS(Robbie);
