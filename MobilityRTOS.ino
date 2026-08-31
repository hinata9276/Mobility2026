/*
Reinforcement Learning Line Follower
2026 TARUMT Mobility Programme

Firmware designer : Dr Lee Yang Yang
Last modified     : 31st August 2026
Version           : 1.12
Operating System  : FreeRTOS

Description:
This program implements a tabular Q-learning algorithm on an ESP32
to control a 5-sensor line-following robot.

STATE:
The five IR sensors are encoded as a 5-bit binary value, giving
2^5 = 32 possible states (0-31). The sensor arrangement is:

       L2   L1   C0   R1   R2

A black-line detection is represented by 1 and no line detection
by 0. The resulting 5-bit pattern is used directly as the Q-table
state index.

ACTION:
Five possible motor actions are defined:
       0 = Hard Left
       1 = Left
       2 = Centre
       3 = Right
       4 = Hard Right

Q-TABLE:
QT[32][5] stores the estimated long-term reward for performing
each action in each sensor state:

       QT[state][action]

The Q-table is initially zero, meaning that the robot has no
prior knowledge of which actions are desirable.

REWARD:
The reward is determined from the new sensor state after an action.
A centred robot receives the highest reward, while deviation from
the centre receives progressively lower rewards. Losing the line
receives a strong negative reward.

Q-LEARNING:
After executing an action, the robot observes the new state and
updates the corresponding Q-value using:

Q(s,a) = Q(s,a) + alpha * [r + gamma * max Q(s',a') - Q(s,a)]

where:
  alpha = learning rate
  gamma = discount factor
  r     = reward received
  s     = previous state
  a     = selected action
  s'    = new state

EXPLORATION / EXPLOITATION:
An epsilon-greedy policy is used. With probability epsilon, the
robot selects a random action to explore the environment. Otherwise,
it selects the action with the highest Q-value. Epsilon gradually
decreases during training so that the robot transitions from
exploration to exploitation as it learns.

FREERTOS:
The ESP32 uses separate FreeRTOS tasks for:
   - LED status indication
   - IR sensor acquisition
   - Q-learning
   - Motor control
More tasks could be added on the later stage without worrying 
messing up the program sequence. Each task is contained in 
its own memory space. 2 CPU cores could be exploited for maximum 
performance. Mutex should be implemented but only one task is 
writing each global variable so it should be fine.

The resulting system allows the robot to learn a line-following
policy directly from its sensor feedback without explicitly
programming a fixed steering rule.
*/
#include <math.h>
#include <EEPROM.h>

//------------- constant definition ---------------
#if CONFIG_FREERTOS_UNICORE
#define TASK_RUNNING_CORE 0 //(don't touch)
#else
#define TASK_RUNNING_CORE 1 //(don't touch)
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // (don't touch)
#endif

//Define pin assignments (don't touch)
#define IR0 13
#define IR1 12
#define IR2 14
#define IR3 27
#define IR4 26
#define MLA 18
#define MLB 5
#define MRA 19
#define MRB 21

//Define Q-Table size and RL parameters
#define NUM_STATES    32    // (don't touch)
#define NUM_ACTIONS   5     // (don't touch)
#define ALPHA         0.1   // learning rate
#define GAMMA         0.9   // discount factor
#define EPSILON       1     // exploration rate, start with full 
#define EPSILON_MIN   0.05  // exploration rate never go to zero
#define EPSILON_DECAY 0.999 // exploration decay factor

//Define motor speed
#define LOW_SPEED     80
#define MID_SPEED     120
#define HI_SPEED      140

//Define global delay start, tone, and LED blink period
#define DELAY_START   3000  //in ms,
#define PLAY_TONE     1     //1= play startup tone, 0 tone off
#define BLINK_DELAY   1000  //default LED blinking period

//Define EEPROM size  (don't touch)
#define EEPROM_SIZE   1024
#define ARRAY_ROWS    32
#define ARRAY_COLS    5
#define ARRAY_SIZE    (ARRAY_ROWS * ARRAY_COLS)
#define ARRAY_BYTES   (ARRAY_SIZE * sizeof(float))
// EEPROM layout  (don't touch)
#define ADDR_ARRAY    0 
#define ADDR_COUNT    (ADDR_ARRAY + ARRAY_BYTES)
#define ADDR_VERSION  (ADDR_COUNT + sizeof(uint32_t))
//EEPROM write control
#define WRITE_ENABLE  1  //1=enable, 0=disable
#define WRITE_CYCLE   10  //max EEPROM write cycle, can only be reset with new firmware
#define RESET_QTABLE  0
//1=reset Q-table memory upon firmware update
//0=keep Q-table memory

/*--------------------------------------------------*/
/*----------- Global Variables Definition ----------*/
/*--------------------------------------------------*/
// This changes whenever the sketch is recompiled.
// Therefore, a new upload starts with 0 writes again.
const char *BUILD_ID = __DATE__ " " __TIME__;
uint32_t writeCount = 0;

//for motor control
const int freq = 5000;    // 5 kHz PWM frequency 
const int resolution = 8; // 8-bit resolution (0-255)
unsigned short SLA,SLB,SRA,SRB; //motor speed

//for Q-learning and state feedback
volatile unsigned short State = 4;  //initialise some value
volatile unsigned short Action = 2; //for Q-Table indexing
volatile float epsilonBlink = EPSILON;
float QT[NUM_STATES][NUM_ACTIONS];
  //Q-table, only anticipated 10 possible states, but allocated extra for unexpected sensor input
  //       |               Action
  //State  | H-Left  Left  Centre  Right H-Right
  //0      | score
  //1      |         score
  //...    |                score
  //31     |                       score

//for serial print sharing mutex
SemaphoreHandle_t serialMutex = NULL;
SemaphoreHandle_t pwmMutex = NULL;

//------------- task name prototyping ---------------
// Define tasks name for FreeRTOS
void TaskBlink(void *pvParameters);
void TaskDigitalRead(void *pvParameters);
void TaskRL(void *pvParameters);
void TaskMotorControl(void *pvParameters);
void TaskEEPROM(void *pvParameter);
void TaskTone(void *pvParameter);
TaskHandle_t digitalRead_task_handle;  // You can  use this to be able to manipulate a task from somewhere else.

/*--------------------------------------------------*/
/*--------------- One-Time Setup -------------------*/
/*--------------------------------------------------*/

void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  pinMode(IR0,INPUT);
  pinMode(IR1,INPUT);
  pinMode(IR2,INPUT);
  pinMode(IR3,INPUT);
  pinMode(IR4,INPUT);
  
// Configure PWM pins
  ledcAttach(MLA, freq, resolution);
  ledcAttach(MLB, freq, resolution);
  ledcAttach(MRA, freq, resolution);
  ledcAttach(MRB, freq, resolution);
  SLA = 0; SLB = 0; SRA = 0; SRB = 0; //0-255 PWM
  ledcWrite(MLA, 0);ledcWrite(MRA, 0);
  ledcWrite(MLB, 0);ledcWrite(MRB, 0);

//serial print info
  startUpPrint();
  EEPROM.begin(EEPROM_SIZE);
  // Check whether this is a new uploaded firmware
  checkFirmwareVersion();
 
  Serial.print("\nESP32 EEPROM storage\n");
  Serial.printf("Build: %s\n", BUILD_ID);
  Serial.printf("Write count: %lu / 5\n",
                  (unsigned long)writeCount);

  // Set up FreeRTOS tasks to run independently.
  // initialise semaphore
  serialMutex = xSemaphoreCreateMutex(); // Initialize it first!
  pwmMutex = xSemaphoreCreateMutex(); // Initialize it first!
  xTaskCreate(//Task Blink
    TaskBlink,            // Task function
    "Task Blink",         // Task name
    1024,                 // Stack size
    NULL,                 // Parameter
    3,                    // Priority, high
    NULL                  // Task handle
  );
  xTaskCreate(//Task Reinforcement Learning
    TaskRL,               // Task function
    "Task Reinforcement Learning",         // Task name
    4096,                 // Stack size
    NULL,                 // Parameter
    3,                    // Priority, high
    NULL                  // Task handle
  );
  xTaskCreate(//Task Motor Control
    TaskMotorControl,     // Task function
    "Task Motor Control", // Task name
    2048,                 // Stack size
    NULL,                 // Parameter
    2,                    // Priority, medium
    NULL                  // Task handle
  );
  xTaskCreate(//Task EEPROM management
    TaskEEPROM,       // Task function
    "EEPROM Task",    // Task name
    2048,             // Stack size
    NULL,             // Parameter
    1,                // Priority, low
    NULL              // Task handle
  );
  xTaskCreate(//Task tone
    TaskTone,         //tas function
    "Task pley Mario tone",
    4096,             //stack size
    NULL,             //parameter
    1,                //priority, low
    NULL              //task handle
  );
  xTaskCreatePinnedToCore(//Task Digital Read, pinned to core 1
    TaskDigitalRead,    // Task function
    "Task Read IR",     // Task name
    2048,               // Stack size
    NULL,               // Parameter
    3,                  // Priority, high
    &digitalRead_task_handle,  // Task handle
    TASK_RUNNING_CORE   // Core on which the task will run
  );
  // Now the FreeRTOS task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop() {}//nothing to run for now

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters) {  // to indicate the system is running
  uint32_t blink_delay = BLINK_DELAY;

  // initialize digital LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  for (;;){                               // A Task shall never return or exit.
    //when epsilonBlink drops, it changes the turn on time to fast flash
    digitalWrite(LED_BUILTIN, HIGH);      // turn the LED on (HIGH is the voltage level)
    // arduino-esp32 has FreeRTOS configured to have a tick-rate of 1000Hz and portTICK_PERIOD_MS
    // refers to how many milliseconds the period between each ticks is, ie. 1ms.
    vTaskDelay(blink_delay*epsilonBlink); //yield to the RTOS
    digitalWrite(LED_BUILTIN, LOW);       // turn the LED off by making the voltage LOW
    vTaskDelay(blink_delay);              //yield to the RTOS
  }
}

void TaskDigitalRead(void *pvParameters) {  //sensing task
  (void)pvParameters;
  unsigned short C0,L1,L2,R1,R2;//variable to local task

  for (;;){
    L2 = 1-digitalRead(IR0);
    L1 = 1-digitalRead(IR1);
    C0 = 1-digitalRead(IR2);
    R1 = 1-digitalRead(IR3);
    R2 = 1-digitalRead(IR4);
    // IR sensor array = L2 L1 C0 R1 R2, encode into binary value
    State = (L2 << 4) | (L1 << 3) | (C0  << 2) |  (R1 << 1) |  (R2);
    /*
    00000 → State 0
    00001 → State 1
    00011 → State 3
    00010 → State 2
    00110 → State 6
    00100 → State 4
    01100 → State 12
    01000 → State 8
    11000 → State 24
    10000 → State 16
  	*/
    //other task might be using the serial, wait for the
    //serial to be released by other task 1st
    //if over 10ms delay, give up mutex and skip printing
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10))) {
       Serial.printf("%d %d %d %d %d \n", L2,L1,C0,R1,R2);
       xSemaphoreGive(serialMutex);
    } 
    vTaskDelay(10);  //yield to the RTOS
  }
}

void TaskRL(void *pvParameters) {  // Reinforcement learning task
  (void)pvParameters;
  unsigned short formerState,currentState;
  float alpha   = ALPHA;      // learning rate
  float gamma   = GAMMA;      // discount factor
  float epsilon = EPSILON;    // exploration rate
  float epsilonMin = EPSILON_MIN;
  float epsilonDecay = EPSILON_DECAY;
  int reward;
  vTaskDelay(DELAY_START);    //yield to the RTOS, wait for user

  for (;;){ // A Task shall never return or exit.
    //State is a global variable
    //State is managed by TaskDigitalRead() task, 
    //so the State will always be updated via RTOS scheduling
    formerState = State;
    //Action is a global variable, output is reflected on TaskMotorControl() task
    Action = chooseAction(formerState,epsilon);
    vTaskDelay(100);                    //yield to the RTOS, wait for new state
    currentState = State;               //get new state
    reward = getReward(currentState);   //compute reward

    //update Q-table, alpha = learn rate, gamma = discount factor
    QT[formerState][Action] = QT[formerState][Action] + alpha * 
    (reward + gamma * maxQ(currentState) - QT[formerState][Action]);

    //epsilon = exploration rate, epsilonDecay = exploration rate decay
    if(epsilon > epsilonMin){epsilon *= epsilonDecay;}
    else{epsilon = epsilonMin;}
    epsilonBlink = epsilon;             //update LED status
    vTaskDelay(10);                     //yield to the RTOS
  }
}

void TaskMotorControl(void *pvParameters) {  //motor control task
  (void)pvParameters;
  //0 = hard left
  //1 = left
  //2 = centre
  //3 = right
  //4 = hard right
  //ledcWrite(MLA, SLA); Speed vary from 0 ~ 255
  vTaskDelay(DELAY_START);//yield to the RTOS, wait for user
  for (;;){  
    switch(Action){
      case 0: SLA = LOW_SPEED;   SRA = HI_SPEED; break;
      case 1: SLA = MID_SPEED;   SRA = HI_SPEED; break;
      case 2: SLA = HI_SPEED;    SRA = HI_SPEED; break;
      case 3: SLA = HI_SPEED;    SRA = MID_SPEED; break;
      case 4: SLA = HI_SPEED;    SRA = LOW_SPEED; break;
      default: SLA = 0;  SRA = 0; break;
    }
    if(xSemaphoreTake(pwmMutex, pdMS_TO_TICKS(10))){
      ledcWrite(MLA, SLA);      //set motor PWM
      ledcWrite(MRA, SRA);
      xSemaphoreGive(pwmMutex);
    }
    vTaskDelay(10);           //yield to the RTOS
  }
}

void TaskEEPROM(void *pvParameters){
  (void)pvParameters;
  bool writeEnable = WRITE_ENABLE;
  for(;;){
    vTaskDelay(60000); //delay 60 seconds 1st
    // Stop permanently after 5 writes
    if(writeCount >= WRITE_CYCLE){
      if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
        Serial.println("EEPROM write limit reached.");
        Serial.println("Waiting for new firmware upload...");
        xSemaphoreGive(serialMutex);
      }
      continue;       //repeat the loop
    }
    else if(!writeEnable){
      if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
        Serial.println("EEPROM write has been disabled");
        xSemaphoreGive(serialMutex);
      }
      continue;       //repeat the loop
    }
    // Save to EEPROM
    saveArray(QT);
    if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
      Serial.printf("EEPROM write #%lu completed\n",
                    (unsigned long)writeCount);
      xSemaphoreGive(serialMutex);
    }
  }
}

void TaskTone(void *pvParameters){
  // Melody structure
  struct Note {
    note_t note;
    uint8_t octave;
    uint16_t duration;
  };
  // Super Mario Bros. opening
  Note melody[] = {
    {NOTE_E,   5, 250},
    {NOTE_E,   5, 250},
    {NOTE_MAX, 0, 125},   // REST
    {NOTE_E,   5, 125},
    {NOTE_MAX, 0, 125},   // REST
    {NOTE_C,   5, 125},
    {NOTE_E,   5, 125},
    {NOTE_MAX, 0, 125},   // REST
    {NOTE_G,   5, 250},
    {NOTE_MAX, 0, 250},   // REST
    {NOTE_G,   4, 250}
  };
  const int numNotes = sizeof(melody) / sizeof(melody[0]);
  for(;;){
    if(xSemaphoreTake(pwmMutex, pdMS_TO_TICKS(10)) && PLAY_TONE){
      // Play melody
      for (int i = 0; i < numNotes; i++){
        if (melody[i].note == NOTE_MAX){
          // Rest
          ledcWriteTone(MLA, 0);
          ledcWriteTone(MRA, 0);
        }
        else{
          // Play note
          ledcWriteNote(MLA,melody[i].note,melody[i].octave);
          ledcWriteNote(MRA,melody[i].note,melody[i].octave);
        }
        // Wait without occupying the CPU
        vTaskDelay(melody[i].duration);
      }
      // Turn buzzer off
      ledcWriteTone(MLA, 0);
      ledcWriteTone(MRA, 0);
    }
    //release pwm hardware
    xSemaphoreGive(pwmMutex);
    vTaskDelete(NULL);
    //remove the task to save CPU resources
  }
}

/*--------------------------------------------------*/
/*------------- Function Definition ----------------*/
/*--------------------------------------------------*/
//functions for Q-learning
int getReward(unsigned short state){
  int error,reward;
  switch(state){
    case 16: error = -4; break; //10000 → State 16
    case 24: error = -3; break; //11000 → State 24
    case 28: error = -2; break; //11100 → State 28
    case 8:  error = -2; break; //01000 → State 8
    case 12: error = -1; break; //01100 → State 12
    case 14: error = 0; break;  //01110 → State 14
    case 4:  error = 0; break;  //00100 → State 4
    case 6:  error = 1; break;  //00110 → State 6
    case 7:  error = 2; break;  //00111 → State 7
    case 2:  error = 2; break;  //00010 → State 2
    case 3:  error = 3; break;  //00011 → State 3
    case 1:  error = 4; break;  //00001 → State 1
    case 0:  error = 10; break;  //00000 → State 0
    default: error = 5; break;
  }
  reward = 10 - abs(error) * 3;
  return reward;
}
float maxQ(unsigned short state){
  float maximum = QT[state][0];
  for(int a = 1; a < NUM_ACTIONS; a++){
    if(QT[state][a] > maximum){
      maximum = QT[state][a];
    }
  }
  return maximum;
}
int getBestAction(unsigned short state){
  int best = 0;
  for(int a = 1; a < NUM_ACTIONS; a++){
    if(QT[state][a] > QT[state][best]){
      best = a;
    }
  }
  return best;
}
int chooseAction(unsigned short state, float epsilon){
  if(random(1000) < epsilon * 1000){// Exploration
    return random(NUM_ACTIONS);
  }
  else{// Exploitation
    return getBestAction(state);
  }
}
// --------------------------------------------------
// Save the 32x5 float array
// --------------------------------------------------
void saveArray(float data[ARRAY_ROWS][ARRAY_COLS]){
  EEPROM.put(ADDR_ARRAY, data);
  writeCount++;
  EEPROM.put(ADDR_COUNT, writeCount);
  // Store firmware/build identifier
  char version[32] = {0};
  strncpy(version, BUILD_ID, sizeof(version) - 1);
  EEPROM.put(ADDR_VERSION, version);
  EEPROM.commit();
}
// --------------------------------------------------
// Check whether this is a new firmware upload.
// No serial mutex is needed because it is called
// in the setup() only for once, not fighting for
// serial port with the RTOS tasks.
// --------------------------------------------------
void checkFirmwareVersion(){
  char storedVersion[32] = {0};
  EEPROM.get(ADDR_VERSION, storedVersion);
  // Load previously stored data 
  EEPROM.get(ADDR_ARRAY, QT);
  EEPROM.get(ADDR_COUNT, writeCount);
  //check firmware version
  if (strcmp(storedVersion, BUILD_ID) != 0){
    // New firmware detected
    Serial.println("New firmware detected.");
    Serial.println("Resetting EEPROM write counter.");
    writeCount = 0;
    EEPROM.put(ADDR_COUNT, writeCount);
    //check if user require Q table reset
    if(RESET_QTABLE){
      Serial.println("Resetting EEPROM Q table.");
      memset(QT, 0, sizeof(float) * NUM_STATES * NUM_ACTIONS);
      EEPROM.put(ADDR_ARRAY, QT);
    }
    else{
      Serial.println("EEPROM Q table unchanged.");
    }
    char version[32] = {0};
    strncpy(version, BUILD_ID, sizeof(version) - 1);
    EEPROM.put(ADDR_VERSION, version);
    EEPROM.commit();
  }
  else{
    Serial.printf("Existing firmware. Write count = %lu\n",
                 (unsigned long)writeCount);
    Serial.println("EEPROM Q table unchanged.");
  }
}
void startUpPrint(){
  Serial.println("Reinforcement Learning Line Follower");
  Serial.println("2026 TARUMT Mobility Programme");
  Serial.println("Firmware designer: Dr Lee Yang Yang");
  Serial.println("Last modified: 31st August 2026");
  Serial.println("Version: 1.12");
  Serial.println("Operating System: FreeRTOS");
}