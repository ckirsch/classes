#include <conio.h>
#include <unistd.h>
#include <dsound.h>
#include <dsensor.h>
#include <unistd.h>
#include <dmotor.h>
#include <lnp.h>
#include <lnp-logical.h>
#include <string.h>

/*  defines for motor setup */

#define driveSpeed motor_c_speed
#define steerSpeed motor_b_speed
#define liftSpeed motor_a_speed
#define driveDir motor_c_dir
#define steerDir motor_b_dir
#define liftDir motor_a_dir
#define left fwd
#define right rev
#define lift rev
#define drop fwd


/* some constants */


//90 degree turn is 480 rotation ticks

#define TICKS_PER_RAD 306.0
#define TICKS_PER_HALF_CIRCLE 960  //num ticks per 180 degrees
#define TICKS_PER_90_DEGREES 480

#define SETPOINT_ERROR_SQUARED 1 //for x,y controller
#define SETPOINT_ERROR 80   //for travel distance controller (quarter ticks)
#define STEER_SETPOINT_ERROR 5
#define LENGTH 452.0  //length of car (quarter ticks)
#define MAX_STEER_ANGLE_FULL_SPEED  100 //the max steer angle at full drive speed
#define MIN_DRIVE_SPEED 50 //the minimum robot speed

#define RCX_PORT 5
#define DEST_PORT 0x9
#define DEST_HOST 0x1
#define DEST_ADDR (DEST_HOST << 4 | DEST_PORT)


pid_t pid1, pid2, pid3;


void b2();
void b3();
void b4();

//PORTS AND STATE
int controlMode=0;  //the controller mode
int liftState=0;
int x=0;  //state (quarter rotation units)
int y=0;  //state (quarter rotation units)
int theta=0; //state

int volatile thetaSetPoint;
int volatile steerAngleSetPoint=0;  //steer angle set point in 1/16 rotation units
int volatile steerAngle=0;
int volatile prevSteerAngle=0;
int volatile speed = 0;
int volatile steer_speed = 0;

int blockDetected=0;  //are we over a block

int deltaDistance;
int xSetPoint, ySetPoint, travelDistance;
int RCX_RANGE = 0;  // Set IR range on RCX, #define didn't work!

//e machine state
int eState = 0;

//communication
char rcvMsg[12];  		// Data received
int rcvMsgLen = 0;		// Length of received data, zero if nothing to receive
char sendMsg[12];		// Data to send
int sendMsgLen = 0;			// Length of data to send, zero if nothing to send


//UTILITIES
//480 ticks/90 degrees
//306 ticks/rad

//taylor series around zero (theta in ticks)
float sinZero(int theta) {
	float r=theta/TICKS_PER_RAD;
	float rr=r*r;
	return (-(rr*r)/5040.0*(rr*rr)) + ((rr*rr*r)/120.0) - ((rr*r)/6.0) + r;
}

//taylor series around pi (theta in ticks)
float sinPi(int theta) {
	float r,rr;
	if(theta<0) r=(theta+TICKS_PER_HALF_CIRCLE)/TICKS_PER_RAD;
	else r=(theta-TICKS_PER_HALF_CIRCLE)/TICKS_PER_RAD;
	rr=r*r;
	return ((rr*r)/5040.0*(rr*rr)) - ((rr*rr*r)/120.0) + ((rr*r)/6.0) - r;
}


//accepts angles -pi to pi
float sin(int theta) {
	if(theta>-TICKS_PER_90_DEGREES && theta<TICKS_PER_90_DEGREES) return sinZero(theta);
	else return sinPi(theta);
}

//accepts angles -pi to pi
float cos(int theta) {
	if(theta>0) return sinPi(theta+TICKS_PER_90_DEGREES);
	else return sinZero(theta+TICKS_PER_90_DEGREES);
}

//taylor series around zero (theta in ticks)
float tan(int theta) {
	float r=theta/TICKS_PER_RAD;
	float rr=r*r;
	return ((rr*rr*r)/5.0) + ((rr*r)/3.0) + r;
}

//taylor series around zero (returns angle in ticks)
//only accurate from -1 to 1, threshold at pi/2
int arctan(float x) {
	float xx=x*x;
	float result = ((xx*xx*x)/5.0) - ((xx*x)/3.0) + x;
	int ticks = (int)(result*TICKS_PER_RAD);
	if(ticks>TICKS_PER_90_DEGREES) return TICKS_PER_90_DEGREES;
	else if(ticks<-TICKS_PER_90_DEGREES) return -TICKS_PER_90_DEGREES;
	else return ticks;
}



//DRIVERS

// Called each time an addressing packet arrives on receiving port
void handler(const unsigned char* data, unsigned char len, unsigned char src) {
	int i;
	for (i = 0; (i < len); i++) rcvMsg[i] = data[i];
	rcvMsg[i] = '\0';
	rcvMsgLen = len;
}


void sendDriver () {
	if (sendMsgLen != 0) {
		lnp_addressing_write(sendMsg, sendMsgLen, DEST_ADDR, RCX_PORT);	//Sends message across network
		sendMsgLen = 0;
	}
}

void lightDriver() {
	lcd_int(LIGHT_3);
	if(LIGHT_3<55 && blockDetected==0) blockDetected=1;
}

void beepDriver() {
	dsound_system(DSOUND_BEEP);
}

void motorDrivers() {
	if(speed < 0) { 
		driveDir(fwd);
		if(speed>-MIN_DRIVE_SPEED) speed=-MIN_DRIVE_SPEED;
		driveSpeed(-speed);
	} else if(speed > 0){
		driveDir(rev);
		if(speed<MIN_DRIVE_SPEED) speed=MIN_DRIVE_SPEED;
		driveSpeed(speed);
	} else driveSpeed(0);
	
	if(steer_speed < 0) { 
		steerDir(right);
		if(steer_speed>-35) steer_speed=-35;
		steerSpeed(-steer_speed);
	} else if(steer_speed > 0) {
		steerDir(left);
		if(steer_speed<35) steer_speed=35;
		steerSpeed(steer_speed);
	} else {
		steerDir(brake);
		steerSpeed(50);
	}
}

void liftDriver() {
	if(liftState==1) {
		liftSpeed(50);
		liftDir(lift);
	} else if(liftState==0) {
		liftDir(brake);
		liftSpeed(255);
	}
}

void rotationDrivers() {
	deltaDistance = ROTATION_2*4; //quarter rotation units
	ds_rotation_set(&SENSOR_2, 0);
	prevSteerAngle = steerAngle;
	steerAngle = -ROTATION_1;
}


void initDriver() {
	int i;
	liftDir(drop);
	liftSpeed(100);
	driveDir(fwd);
	driveSpeed(0);
	steerSpeed(0);
 	ds_active(&SENSOR_2);
 	ds_rotation_set(&SENSOR_2, 0);
  	ds_rotation_on(&SENSOR_2);
  	ds_active(&SENSOR_1);
 	ds_rotation_set(&SENSOR_1, 0);
  	ds_rotation_on(&SENSOR_1);
  	ds_active(&SENSOR_3);
  	lnp_logical_range(RCX_RANGE); // Sets transmitter range on RCX
  	lnp_addressing_set_handler(RCX_PORT, handler); // Installs a handler on receiving port
  	msleep(3000);
  	liftDir(brake);
  	liftSpeed(200);
}


//TASKS

void liftTask() {
	speed=0;
	liftState=1;
}

void returnTask() {
	liftState=0;
	xSetPoint=0;
	ySetPoint=0;
	controlMode=4;
}

int round(float x) {
	if(x>0) return (int)(x+0.5);
	else if(x<0) return(int)(x-0.5);
	else return 0;
}

void stateEstimator() {
	theta = theta + round((deltaDistance/(float)(LENGTH)) * tan((steerAngle+prevSteerAngle)/2) * (float)TICKS_PER_RAD);
	if(theta>TICKS_PER_HALF_CIRCLE) theta-=2*TICKS_PER_HALF_CIRCLE;
	else if(theta<-TICKS_PER_HALF_CIRCLE) theta+=2*TICKS_PER_HALF_CIRCLE;
	x = x + round((float)(deltaDistance)*cos(theta));
	y = y + round((float)(deltaDistance)*sin(theta));
	//lcd_int(deltaDistance);
	exit(0);
}

void sendState() {
	sendMsg[0]=theta;
	sendMsg[1]=(theta>>8);
	sendMsg[2]=x;
	sendMsg[3]=(x>>8);
	sendMsg[4]=y;
	sendMsg[5]=(y>>8);
	sendMsg[6] = '\0';
    sendMsgLen = 7;
    exit(0);
}

void controlModeSwitcher() {
	if(rcvMsgLen!=0) {
		beepDriver();
		controlMode = rcvMsg[0];
		switch(controlMode) {
			case 1:
				steerAngleSetPoint=(rcvMsg[1]<<8) + (unsigned)rcvMsg[2];
				speed=(rcvMsg[3]<<8) + (unsigned)rcvMsg[4];
				break;
			case 2:
				steerAngleSetPoint=(rcvMsg[1]<<8) + (unsigned)rcvMsg[2];
				travelDistance=(rcvMsg[3]<<8) + (unsigned)rcvMsg[4];
				break;
			case 3:
				thetaSetPoint=(rcvMsg[1]<<8) + (unsigned)rcvMsg[2];
				travelDistance=(rcvMsg[3]<<8) + (unsigned)rcvMsg[4];
				break;
			case 4:
			case 5:
				xSetPoint=(rcvMsg[1]<<8) + (unsigned)rcvMsg[2];
				ySetPoint=(rcvMsg[3]<<8) + (unsigned)rcvMsg[4];
				break;
		}
		rcvMsgLen=0;
	}
}


//steering angle controller
//TO DO: D control
void steerAngleController() {
	int error = steerAngleSetPoint - steerAngle;
		if(error > STEER_SETPOINT_ERROR || error < -STEER_SETPOINT_ERROR) {
			error = error/8;
			if(error > 255) steer_speed=255;
			else if(error < -255) steer_speed=-255;
			else steer_speed=error;
		} else steer_speed=0;
}

//theta controller
void thetaController() {
	int thetaError = thetaSetPoint - theta;
	if(thetaError<-TICKS_PER_HALF_CIRCLE) thetaError+=TICKS_PER_HALF_CIRCLE*2;
	else if(thetaError>TICKS_PER_HALF_CIRCLE) thetaError-=TICKS_PER_HALF_CIRCLE*2;
	steerAngleSetPoint = -steerAngleSetPoint/4 + thetaError; //pd control on theta
	if(steerAngleSetPoint>200) steerAngleSetPoint=200;
	else if(steerAngleSetPoint<-200) steerAngleSetPoint=-200;
}

//backward theta controller
void backwardThetaController() {
	int thetaError = thetaSetPoint - theta;
	if(thetaError<-TICKS_PER_HALF_CIRCLE) thetaError+=TICKS_PER_HALF_CIRCLE*2;
	else if(thetaError>TICKS_PER_HALF_CIRCLE) thetaError-=TICKS_PER_HALF_CIRCLE*2;
	steerAngleSetPoint = -steerAngleSetPoint/4 - thetaError; //minus pd control on theta
	if(steerAngleSetPoint>200) steerAngleSetPoint=200;
	else if(steerAngleSetPoint<-200) steerAngleSetPoint=-200;
}


//travel distance controller
//TO DO: scale speed for high steer angle
//TO DO: soft start
void travelDistanceController() {
	int error;
	travelDistance -= deltaDistance;
	error = travelDistance;
	if(error > SETPOINT_ERROR || error<-SETPOINT_ERROR) {
		error = error/8;
		if(error > 150) speed=150;
		else if(error < -150) speed=-150;
		else speed=error;
	} else speed=0;
}

//xy pointing controller
void pointingController(int delX, int delY) {
	int absDelX, absDelY;
	float arc;
	
	if(delY<0) absDelY=-delY;
	else if(delY>0) absDelY=delY;
	else if(delX>0) { thetaSetPoint=0; return; } 
	else { thetaSetPoint=TICKS_PER_HALF_CIRCLE; return; }
	
	if(delX<0) absDelX=-delX;
	else if(delX>0) absDelX=delX;
	else if(delY>0) { thetaSetPoint = TICKS_PER_90_DEGREES; return; }
	else { thetaSetPoint = -TICKS_PER_90_DEGREES; return; }
			
	if(absDelY<absDelX) { 
		arc = (float)delY/(float)delX;
		thetaSetPoint=arctan(arc);
		if(delX<0) thetaSetPoint+=TICKS_PER_HALF_CIRCLE;	
	} else {
		arc = (float)delX/(float)delY;
		thetaSetPoint=arctan(arc);
		if(delY<0) thetaSetPoint=-TICKS_PER_90_DEGREES-thetaSetPoint;
		else thetaSetPoint=TICKS_PER_90_DEGREES-thetaSetPoint;
	}

}

void controller() {
	long error=0;
	int delX, delY;
	
	switch(controlMode) {
		case 0:  //stop
			steer_speed=0;
			speed=0;
			break;
			
		case 2: //const. steer angle, travel distance
			travelDistanceController();
		
		//fall through
		case 1:  //const. steer angle, const. speed
			steerAngleController();
			break;
			
		case 3:  //const.theta, drive for travel distance.
			travelDistanceController();
			thetaController();
			lcd_int(steerAngleSetPoint);
			steerAngleController();
			lcd_int(steerAngleSetPoint);
			break;
			
		case 4:
			delX=xSetPoint-x;
			delY=ySetPoint-y;
			pointingController(delX, delY);
			delX=delX/4;
			delY=delY/4;
			if(delX>150 || delX<-150 || delY<-150 || delY>150) speed=150;
			else {
				error = (delX/16 * delX/16) + (delY/16 * delY/16);
				if(error > 300) speed=150;
				else if(error < 120) speed=60;
				else speed=error/2;
				if(error < SETPOINT_ERROR_SQUARED) { speed=0; controlMode=0; }
			}

			if(error > SETPOINT_ERROR_SQUARED && speed==0) speed=50;  //soft start
			
			thetaController();
			steerAngleController();
			break;
		case 5: //navigate backwards to pick up blocks!
			delX=xSetPoint-x;
			delY=ySetPoint-y;
			pointingController(delX, delY);
			delX=delX/4;
			delY=delY/4;
			//backwards
			thetaSetPoint-=TICKS_PER_HALF_CIRCLE;
			if(thetaSetPoint<-TICKS_PER_HALF_CIRCLE) thetaSetPoint+=TICKS_PER_HALF_CIRCLE*2;
			
			if(delX>150 || delX<-150 || delY<-150 || delY>150) speed=-150;
			else {
				error = (delX/16 * delX/16) + (delY/16 * delY/16);
				if(error > 300) speed=-150;
				else if(error < 120) speed=-60;
				else speed=-error/2;
				if(error < SETPOINT_ERROR_SQUARED) { speed=0; controlMode=0; }
			}
			
			backwardThetaController();
			steerAngleController();
			break;
	}
	
	//lcd_int(steerAngle);
	exit(0);
}


//E MACHINE

void call(void (*ptToFunc) ()) {
	ptToFunc();
}

void future(int timeTrigger, void (*ptToFunc) ()) {
	msleep(timeTrigger);
	execi(ptToFunc, 0, NULL, 5, DEFAULT_STACK_SIZE);
	exit(0);
}


void branchingFuture(int timeTrigger, int(*ptToPredicate) (), void(*ptToTrueFunc) (), void(*ptToFalseFunc) ()) {
	msleep(timeTrigger);
	if(ptToPredicate()) execi(ptToTrueFunc, 0, NULL, 5, DEFAULT_STACK_SIZE);
	else execi(ptToFalseFunc, 0, NULL, 5, DEFAULT_STACK_SIZE);
	exit(0);
}

void schedule(void (*ptToFunc) (), int priority) {
	execi(ptToFunc, 0, NULL, priority, DEFAULT_STACK_SIZE);
}



//E MACHINE PROGRAM
int programCount() {
	if(eState==0) eState=9;
	else eState-=1;
	return eState;
}

int blockDetector() {   //debouced
	if(blockDetected==1) blockDetected=2;
	else return 0;
	return blockDetected;
}


void b1() {
	call(&motorDrivers);
	call(&rotationDrivers);
	schedule(&controlModeSwitcher, 9);
	schedule(&stateEstimator, 8);
	schedule(&sendState, 7);
	future(50, &b4);
}

void b4() {
	call(&rotationDrivers);
	call(&sendDriver);
	schedule(&stateEstimator, 8);
	schedule(&controller, 7);
	future(50, &b2);
}

void b7() {
	call(&motorDrivers);
	call(&liftDriver);
	call(&rotationDrivers);
	schedule(&stateEstimator, 8);
	schedule(&controller, 7);
	branchingFuture(50, &programCount,&b2, &b1);
}

void b6() {
	call(&motorDrivers);
	call(&liftDriver);
	call(&rotationDrivers);
	schedule(&stateEstimator, 9);
	schedule(&returnTask, 8);
	future(3000, &b7);
}

void b5() {
	call(&rotationDrivers);
	schedule(&stateEstimator, 9);
	schedule(&liftTask, 8);
	future(50, &b6);
}


void b2() {
	call(&motorDrivers);
	call(&rotationDrivers);
	call(&lightDriver);
	schedule(&stateEstimator, 8);
	branchingFuture(50, &blockDetector, &b5, &b3);
}

void b3() {
	call(&rotationDrivers);
	schedule(&stateEstimator, 8);
	schedule(&controller, 7);
	branchingFuture(50, &programCount,&b2, &b1);
}


/*
void b1() {
	call(&motorDrivers);
	call(&rotationDrivers);
	call(&sendDriver);
	schedule(&controlModeSwitcher, 9);
	schedule(&stateEstimator, 8);
	schedule(&controller, 7);
	//beepDriver();
	future(200, &b2);
}

void b2() {
	call(&motorDrivers);
	call(&rotationDrivers);
	schedule(&stateEstimator, 8);
	schedule(&controller, 7);
	future(200, &b3);
}

void b3() {
	call(&motorDrivers);
	call(&rotationDrivers);
	schedule(&stateEstimator, 8);
	schedule(&controller, 7);
	future(200, &b4);
}

void b4() {
	call(&motorDrivers);
	call(&rotationDrivers);
	schedule(&stateEstimator, 8);
	schedule(&controller, 7);
	schedule(&sendState, 6);
	future(200, &b1);
}
*/

void bInit() {
	call(&initDriver);
	execi(&b1, 0, NULL, 6, DEFAULT_STACK_SIZE);
}


//the main block


int main(int argc, char **argv) {
	int i;

	bInit();
	
	speed=0;
	xSetPoint=0;
	ySetPoint=0;
	
	/*for(i=0;i<10;i+=1) {
		while(controlMode!=0) sleep(2);
		if(blockDetected!=0) break;
		beepDriver();
		if(i%2==0) {
			xSetPoint=-5200;
			ySetPoint=i*1000;
		} else { 
			xSetPoint=0;
			ySetPoint=i*1000;
		}
		controlMode=5;
		sleep(2);
	}*/
	
	
	while(1) { 
		sleep(250);
	}
}
