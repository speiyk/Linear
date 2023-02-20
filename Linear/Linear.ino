/**
 * @file .ino
 *
 * Project Linear 2 Axis DC Motor.
 *
 * @author  S.Sarawut
 * @date    2223/02/20
 */
 
#define PIN_PWM_AXIS_X 27
#define PIN_PWM_AXIS_Y 14
#define PIN_MOTOR_FORWARD_AXIS_X 32
#define PIN_MOTOR_REVERSE_AXIS_X 33
#define PIN_MOTOR_FORWARD_AXIS_Y 12
#define PIN_MOTOR_REVERSE_AXIS_Y 13

#define PIN_ENCODE_OUT_A_AXIS_X 23
#define PIN_ENCODE_OUT_B_AXIS_X 22

#define PIN_SAFETY_START_AXIS_X 18
#define PIN_SAFETY_STOP_AXIS_X 	5
#define PIN_SAFETY_START_AXIS_Y 21
#define PIN_SAFETY_STOP_AXIS_Y 	19

#define PIN_TEST_MENU0 4

const int pwm_channel_axis_x = 0;
const int pwm_channel_axis_y = 0;
const int frequence = 5000;
const int resolution = 8;
volatile int counter; 
int present_state;
int previous_state;

uint8_t step[100];
uint8_t ix_status = 0;
uint8_t ix_step = 0;

int p_safety_stopx;
int p_safety_startx;
int p_safety_stopy;
int p_safety_starty;

bool running = false;
int wait_2s = 0;

void motor_stop_axis_x() { 
  digitalWrite(PIN_MOTOR_FORWARD_AXIS_X, LOW);
  digitalWrite(PIN_MOTOR_REVERSE_AXIS_X, LOW);
  ledcWrite(pwm_channel_axis_x, 00);
}

void motor_forward_axis_x(uint8_t speedx) { 
  if(p_safety_stopx == LOW) {
  	digitalWrite(PIN_MOTOR_FORWARD_AXIS_X, HIGH);
  	digitalWrite(PIN_MOTOR_REVERSE_AXIS_X, LOW);
  	ledcWrite(pwm_channel_axis_x, speedx); 
  }
  else {
	  motor_stop_axis_x();  
  }	  
}

void motor_reverse_axis_x(uint8_t speedx) { 
  if(p_safety_startx == LOW) {
  	digitalWrite(PIN_MOTOR_FORWARD_AXIS_X, LOW);
  	digitalWrite(PIN_MOTOR_REVERSE_AXIS_X, HIGH);
  	ledcWrite(pwm_channel_axis_x, speedx);
  }
  else {
	  motor_stop_axis_x();
  }
}

void motor_stop_axis_y() { 
  digitalWrite(PIN_MOTOR_FORWARD_AXIS_Y, LOW);
  digitalWrite(PIN_MOTOR_REVERSE_AXIS_Y, LOW);
  ledcWrite(pwm_channel_axis_y, 00);
}

void motor_forward_axis_y(uint8_t speedy) {  
  if(p_safety_stopy == LOW) {
    digitalWrite(PIN_MOTOR_FORWARD_AXIS_Y, HIGH);
    digitalWrite(PIN_MOTOR_REVERSE_AXIS_Y, LOW);
    ledcWrite(pwm_channel_axis_y, speedy);
  }
  else {
	  motor_stop_axis_y();
  }
}

void motor_reverse_axis_y(uint8_t speedy) { 
  if(p_safety_starty == LOW) {
  	digitalWrite(PIN_MOTOR_FORWARD_AXIS_Y, LOW);
  	digitalWrite(PIN_MOTOR_REVERSE_AXIS_Y, HIGH);
  	ledcWrite(pwm_channel_axis_y, speedy);
  }
  else {
	  motor_stop_axis_y();
  }
}

void encoder(){
  static int lastA = 0;
  int currentA = digitalRead(PIN_ENCODE_OUT_A_AXIS_X);
  int currentB = digitalRead(PIN_ENCODE_OUT_B_AXIS_X);
  
  if ((lastA == LOW) && (currentA == HIGH)) {
    if (currentB == LOW) {counter++;} 
	  else {counter--;}
  }
  
  lastA = currentA;
}

void setup() { 
  pinMode (PIN_TEST_MENU0, INPUT);
  
  pinMode (PIN_ENCODE_OUT_A_AXIS_X, INPUT);
  pinMode (PIN_ENCODE_OUT_B_AXIS_X, INPUT); 
  pinMode (PIN_SAFETY_START_AXIS_X, INPUT);
  pinMode (PIN_SAFETY_STOP_AXIS_X,  INPUT);
  pinMode (PIN_SAFETY_START_AXIS_Y, INPUT);
  pinMode (PIN_SAFETY_STOP_AXIS_Y,  INPUT);
  pinMode(PIN_MOTOR_FORWARD_AXIS_X, OUTPUT);
  pinMode(PIN_MOTOR_REVERSE_AXIS_X, OUTPUT);
  pinMode(PIN_MOTOR_FORWARD_AXIS_Y, OUTPUT);
  pinMode(PIN_MOTOR_REVERSE_AXIS_Y, OUTPUT);
  
  ledcAttachPin(PIN_PWM_AXIS_X, pwm_channel_axis_x);
  ledcAttachPin(PIN_PWM_AXIS_Y, pwm_channel_axis_y);
  ledcSetup(pwm_channel_axis_x, frequence, resolution);
  ledcSetup(pwm_channel_axis_y, frequence, resolution);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODE_OUT_A_AXIS_X), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODE_OUT_B_AXIS_X), encoder, CHANGE);

  Serial.begin (115200); delay(1000);
  
  while (digitalRead(PIN_SAFETY_START_AXIS_X) == LOW) {motor_reverse_axis_x(150); delay(15);} motor_stop_axis_x();
  while (digitalRead(PIN_SAFETY_START_AXIS_Y) == LOW) {motor_reverse_axis_y(255); delay(15);} motor_stop_axis_y();
  
  counter = 0;
  
  Serial.println(" - start v");
}

/*
uint8_t dutyCycle = 0;

void loop() {
  int p_meun0 = digitalRead(PIN_TEST_MENU0);
  
  p_safety_stopx  = digitalRead(PIN_SAFETY_STOP_AXIS_X);
  p_safety_startx = digitalRead(PIN_SAFETY_START_AXIS_X); 
  p_safety_stopy  = digitalRead(PIN_SAFETY_STOP_AXIS_Y);
  p_safety_starty = digitalRead(PIN_SAFETY_START_AXIS_Y);

  //motor_reverse_axis_x(200);
  motor_reverse_axis_y(255);
  delay(2000);

  //motor_forward_axis_x(200);
  motor_forward_axis_y(255);
  delay(2000);

  Serial.println(" safety:" + String(p_safety_stopx) + String(p_safety_startx) + String(p_safety_stopy) + String(p_safety_starty) + " SW:" +  String(p_meun0) + " counter:" + String(counter / 191) + " dutyCycle:" + String(dutyCycle));
  //delay(100);
}*/
  
void loop() {
	double p_count = counter / 191;
  
	int p_meun0 = digitalRead(PIN_TEST_MENU0);
	
	p_safety_stopx  = digitalRead(PIN_SAFETY_STOP_AXIS_X);
	p_safety_startx = digitalRead(PIN_SAFETY_START_AXIS_X);	
	p_safety_stopy  = digitalRead(PIN_SAFETY_STOP_AXIS_Y);
	p_safety_starty = digitalRead(PIN_SAFETY_START_AXIS_Y);
	
	if (p_meun0 == LOW and running == false) {
		//step == 2 END
		//step == 1 forward_axis_y
		step[0] = 10;
		step[1] = 1;
		step[2] = 1;
    step[3] = 30;
    step[4] = 1;
    step[5] = 50;
    step[6] = 1;
		step[7] = 2;
		
		ix_status = 20;
		running = true;
	}
		
	switch (ix_status) {
		case 20: 
			if (p_count >= step[ix_step]){				
				motor_stop_axis_x();
				
				ix_step++;
				ix_status = 30;
			}
			else {
				motor_forward_axis_x(200);
			}				
			break;
		
		case 30:
			if (p_safety_stopy == HIGH) {
				motor_stop_axis_y();
				
				ix_step++;
				ix_status = 40;
			}
			else {
				motor_forward_axis_y(255);
			}
			break;
		
		case 40:
			if (wait_2s == 200) {ix_status = 50; wait_2s = 0;}
			else {wait_2s++;}		
			break;
			
		case 50:
			if (p_safety_starty == HIGH) {
				motor_stop_axis_y();
				
				if (step[ix_step] == 2) {ix_status = 100;}
				else if (step[ix_step] == 1) {ix_status = 30;}
				else {ix_status = 20;}
			}
			else {
				motor_reverse_axis_y(255);
			}
			break;
			
		case 100:
			if (p_safety_startx == HIGH) {
				motor_stop_axis_x();
				
				running = false;
				ix_status = 0;
        ix_step = 0;
			}
			else {
				motor_reverse_axis_x(150);
			}
			break;
			
		default:
			// statements
			break;
	}

	// + "" + String()
	Serial.println(" - ix_status:" + String(ix_status) + " ix_step:" + String(ix_step) + " running:" + String(running) + 
	               " safety:" + String(p_safety_stopx)+ String(p_safety_startx) + String(p_safety_stopy) + String(p_safety_starty) + 
	               " p_count:" + String(p_count));
	
	delay(10);
}
