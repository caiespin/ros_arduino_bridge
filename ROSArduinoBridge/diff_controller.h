/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

#define ENCODER_MAX 16383  // For a 14-bit encoder (2^14 - 1)
#define ENCODER_MIN 0      // Minimum value for the encoder


/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  long PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  long ITerm;                    //integrated term

  long output;                    // last motor setting
  int Kp;                        // Proportional gain
  int Ki;                        // Integral gain
  int Kd;                        // Derivative gain
  int Ko;
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
/*int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;*/
void initPID() {
  // Initialize left motor PID gains
  leftPID.Kp = 20;  // Adjust this value as needed
  leftPID.Ki = 0;   // Adjust this value as needed
  leftPID.Kd = 12;  // Adjust this value as needed
  leftPID.Ko = 50;
  

  // Initialize right motor PID gains
  rightPID.Kp = 20; // Adjust this value as needed
  rightPID.Ki = 0;  // Adjust this value as needed
  rightPID.Kd = 12; // Adjust this value as needed
  rightPID.Ko = 50;
}


unsigned char moving = 0; // is the base in motion?
unsigned char moving_left = 0;  // Left motor movement flag
unsigned char moving_right = 0; // Right motor movement flag


/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

long computeDelta(long current, long previous) {
    long delta = current - previous;

    // Handle wrap-around
    if (delta > (ENCODER_MAX / 2)) {
        // Wrapped around from maximum to minimum
        delta -= (ENCODER_MAX + 1);
    } else if (delta < -(ENCODER_MAX / 2)) {
        // Wrapped around from minimum to maximum
        delta += (ENCODER_MAX + 1);
    }

    return delta;
}


/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  long input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  //input = p->Encoder - p->PrevEnc;
  input = computeDelta(p->Encoder, p->PrevEnc);
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  //output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  // Use individual gains
  output = (p->Kp * Perror - p->Kd * (input - p->PrevInput) + p->ITerm) / p->Ko;
  
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += p->Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  /* If neither motor is moving, there's nothing more to do */
  if (!moving_left && !moving_right) {
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0)
      resetPID();
    return;
  }

  /* Compute PID update for the left motor if it's moving */
  if (moving_left) {
    doPID(&leftPID);
  } else {
    // Ensure the left motor is stopped
    leftPID.output = 0;
    setMotorSpeed(LEFT, 0);
  }

  /* Compute PID update for the right motor if it's moving */
  if (moving_right) {
    doPID(&rightPID);
  } else {
    // Ensure the right motor is stopped
    rightPID.output = 0;
    setMotorSpeed(RIGHT, 0);
  }

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
  /*Serial.print(computeDelta(readEncoder(LEFT), leftPID.PrevEnc));
  Serial.print(" ");
  Serial.print(leftPID.output);
  Serial.print(" ");
  Serial.print(computeDelta(readEncoder(RIGHT), rightPID.PrevEnc));
  Serial.print(" ");
  Serial.println(rightPID.output);*/
}
