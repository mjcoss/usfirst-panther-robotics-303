#include <math.h>
#include "WPILib.h"
#include "Target.h"
#include "Robot.h"

#define MAX_PAN_POSITION    1.0
#define MIN_PAN_POSITION    0.0
#define MAX_TILT_POSITION   0.75
#define MIN_TILT_POSITION   0.4
#define SERVO_STEP			0.05
/*
 * Set servo positions (0.0 to 1.0)
 */
static void 
setServoPositions(float pan, float tilt)
{
	static float servoDeadBand = 0.005;
	double time_remaining;

	// ensure that arguments are within appropriate bounds
	if(pan > MAX_PAN_POSITION)
		pan = MAX_PAN_POSITION;
	else if (pan < MIN_PAN_POSITION)
		pan = MIN_PAN_POSITION;

	if(tilt > MAX_TILT_POSITION)
		tilt = MAX_TILT_POSITION;
	else if (tilt < MIN_TILT_POSITION)
		tilt = MIN_TILT_POSITION;

	time_remaining = robotInfo.next_update - GetTime();
	if(time_remaining > 0)
		taskDelay((int)time_remaining*100);

	if (fabs(pan - robotInfo.pan->Get()) > servoDeadBand)
		robotInfo.pan->Set(pan);
	if (fabs(tilt - robotInfo.tilt->Get()) > servoDeadBand)
		robotInfo.tilt->Set(tilt);

	robotInfo.next_update = GetTime() + 0.02;
}	

static int
Cambot(void)
{
	double savedImageTimestamp = 0.0;
	float pstep, tstep;
	float curH,   curV;
	float panchg, tiltchg, panval, tiltval;
	int follow_threshold;
	TrackingThreshold td1, td2;		// color thresholds
	ParticleAnalysisReport par;		// particle analysis report  
	SecondColorPosition pos;
	
	// PINK
	sprintf(td1.name, "PINK");
	td1.hue.minValue = 220;   
	td1.hue.maxValue = 255;  
	td1.saturation.minValue = 75;   
	td1.saturation.maxValue = 255;      
	td1.luminance.minValue = 85;  
	td1.luminance.maxValue = 255;

	// GREEN
	sprintf(td2.name, "GREEN");
	td2.hue.minValue = 55;   
	td2.hue.maxValue = 125;  
	td2.saturation.minValue = 58;   
	td2.saturation.maxValue = 255;    
	td2.luminance.minValue = 92;  
	td2.luminance.maxValue = 255;
	
	memset(&par, 0, sizeof(par));
	robotInfo.state = SEARCHING;
	pstep = tstep = SERVO_STEP;
	
	pos = (robotInfo.ourColor == DriverStation::kRed) ? ABOVE : BELOW;
	
	dprintf(LOG_DEBUG, "looking for COLOR %s %s %s\n\r", 
			td2.name, (pos == ABOVE) ? "ABOVE" : "BELOW", td1.name);
	
	// wait till servos initialized
	while(!robotInfo.cameraInitialized)
		taskDelay(1);

	while (1) {
		curH = RangeToNormalized(robotInfo.pan->Get(),  1);
		curV = RangeToNormalized(robotInfo.tilt->Get(), 1);
		
		if (FindTwoColors(td1, td2, pos, &par)) {
			if(!tooClose(&par) && bigEnough(&par)) {
				robotInfo.state = TRACKING;
				if (par.imageTimestamp > savedImageTimestamp) {					
					savedImageTimestamp = par.imageTimestamp;	
					panchg  = (par.center_mass_x_normalized/15.0);
					tiltchg = (par.center_mass_y_normalized/-13.0);
					
					curH -= panchg;
					curV -= tiltchg;
					setServoPositions(NormalizeToRange(curH), NormalizeToRange(curV)); 
				}
			}
		} 
		else { // continue search or restart search
			switch(robotInfo.state) {
				case TRACKING: // lost target so we're going to try and go in roughly the same direction
					robotInfo.state = LOST;
					follow_threshold = 3;
					curH -= panchg;
					curV -= tiltchg;
					setServoPositions(NormalizeToRange(curH), NormalizeToRange(curV));
					break;
					
				case LOST:
					if(follow_threshold--) {
						curH -= panchg;
						curV -= tiltchg;
						setServoPositions(NormalizeToRange(curH), NormalizeToRange(curV));
					}
					else {
						// give up on follow mode and restart search
						robotInfo.state = SEARCHING;
						panchg  = 0.0;
						tiltchg = 0.0;
						setServoPositions(MIN_PAN_POSITION, MIN_TILT_POSITION);
					}
					break;
					
				case SEARCHING:
					panval  = NormalizeToRange(curH);
					tiltval = NormalizeToRange(curV);
					// back and forth scan looking for target
					
					panval += pstep;
					if(panval >= MAX_PAN_POSITION) {
						tiltval += tstep;
						if(tiltval >= MAX_TILT_POSITION)
							tstep = -SERVO_STEP;
						else if(tiltval <= MIN_TILT_POSITION)
							tstep = SERVO_STEP;

						pstep = -SERVO_STEP;
						
					}
					if(panval <= MIN_PAN_POSITION) {
						if(tiltval >= MAX_TILT_POSITION)
							tstep = -SERVO_STEP;
						else if(tiltval <= MIN_TILT_POSITION)
							tstep = SERVO_STEP;
						tiltval += tstep;

						pstep = SERVO_STEP;
					}
					
					setServoPositions(panval, tiltval);
					break;
				default:
					// do nothing
					break;
			}
		}
		taskDelay(2);
	}
}

Task handleCamera("cambot", (FUNCPTR)Cambot);

int
initCambot(Servo *pan, Servo *tilt)
{
	bool started=handleCamera.Start();
	
	robotInfo.cameraInitialized = false;
	
	if(started) {
	    pan->Set(MIN_PAN_POSITION);
	    tilt->Set(MIN_TILT_POSITION);
	    
		robotInfo.taskId = handleCamera.GetID();
	    robotInfo.pan    = pan;
	    robotInfo.tilt   = tilt;

	    robotInfo.next_update       = GetTime();
	    robotInfo.cameraInitialized = true;
	    return robotInfo.taskId;
	}
	dprintf(LOG_DEBUG, "Start of Cambot failed");
	return 0;
}

void
StopCambot(void)
{
	if(robotInfo.taskId)
		taskDelete(robotInfo.taskId);
	return;
}
