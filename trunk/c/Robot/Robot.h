#ifndef __ROBOT_H__
#define __ROBOT_H__
#define PI 3.14159265358979

#define NUM_JOYSTICK_BUTTONS			16

#define NUM_DRIVE_MOTORS				2
#define NUM_SHOOTER_MOTORS				2
#define NUM_PICKUP_MOTORS				2
#define GATEKEEPER_MOTOR_BASE_CHAN		7

#define MY_GYRO_SENSITIVITY				0.002

#define MY_ACCELEROMETER_ZERO			1.5
#define MY_ACCELEROMETER_SENSITIVITY	0.04

#define NUM_FREE_WHEEL_ENCODERS			2
#define FREE_WHEEL_DISTANCE_PER_PULSE	(0.333*PI/250.0)
#define FREE_WHEEL_MAX_PERIOD			20
#define FREE_WHEEL_MIN_RATE				1

#define NUM_DRIVE_WHEEL_ENCODERS		2
#define DRIVE_WHEEL_DISTANCE_PER_PULSE	(0.5*PI/250.0/3.0)
#define DRIVE_WHEEL_MAX_PERIOD			20
#define DRIVE_WHEEL_MIN_RATE			1

#define G_FEET 32.1740486       //Value of g in ft/s²
#define JOYSTICK_TRIM 0.01      //Deadzone on the joystick
#define THROTTLE_DEADZONE 0.01  //Deadzone on the throttle

#define TRACTION_CONTROL
#define PROACTIVE_FACTOR				0.2
#define REACTIVE_FACTOR					1000.0

// #define SQUARE_INPUTS

enum Encoder_Side {ENCODER_LEFT=0,ENCODER_RIGHT};

#define ENCODER_DEADBAND				25
#define USE_CAMERA						0						
#define FRAMERATE						15

typedef enum { SEARCHING, TRACKING, LOST, NUM_TRACKING_STATES } tracking_t;
typedef enum { LEFTSIDE, RIGHTSIDE,	NUM_JOYSTICKS } joysticks_t;
typedef enum { AUTO, DISABLED, TELEOP, NUM_OPERATING_MODES } operatorModes_t;
typedef enum { X_AXIS, Y_AXIS, Z_AXIS, NUM_AXISES } axis_t;
typedef enum { NEAR, MIDDLE, FAR } position_t;

#ifdef TRACTION_CONTROL
typedef enum { NO_CONTROL, MIN_CONTROL, MAX_CONTROL, NUM_CONTROL_MODES } tractionModes_t;
#endif

// Some hard coded traction control values, to be used instead of throttle
// NOTE: REACTIVE values are set to off values
#define NO_PROACTIVE					1.0
#define NO_REACTIVE						1.0

#define MIN_PROACTIVE					-0.35
#define MIN_REACTIVE					1.0

#define MAX_PROACTIVE					-0.79
#define MAX_REACTIVE					1.0

typedef struct {
	// accelerometer data
	float		acceleration[NUM_AXISES];
	float		offsets[NUM_AXISES];
	// gyro data
	double		x_heading;
	double		y_heading;
	
	// encoder data
	double		free_wheel_encoders[NUM_FREE_WHEEL_ENCODERS];
	double		drive_wheel_encoders[NUM_DRIVE_WHEEL_ENCODERS];
	// Camera data
	Servo		*pan;
	Servo		*tilt;
	tracking_t	state;
	UINT32		taskId;
	double		next_update;
	// General info
	DriverStation::Alliance	ourColor;
	UINT32		StartingPosition;
	UINT32		pktNum;		// keep track of the most recent packet number from the DS
	UINT8		pktPerSec;	// keep track of the ds packets received in the current second
	bool		cameraInitialized;
#ifdef TRACTION_CONTROL
	tractionModes_t traction;
#endif
} robotStatus_t;

extern robotStatus_t robotInfo;

// function references
extern int initCambot(Servo *, Servo *);
extern void StopCambot(void);
extern bool tooClose(ParticleAnalysisReport*);
extern bool bigEnough(ParticleAnalysisReport*);
extern bool enableGate(float, unsigned int *);
#endif
