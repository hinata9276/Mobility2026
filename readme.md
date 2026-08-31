# Reinforcement Learning Line Follower

### 2026 TARUMT Mobility Programme

**Firmware Designer:** Dr. Lee Yang Yang  
**Last Modified:** 31st August 2026  
**Version:** 1.12  
**Operating System:** FreeRTOS  
**Platform:** ESP32  

---

# 🎯 Objective

The objective of this project is to demonstrate how reinforcement learning can be implemented on a resource-constrained embedded system.

The robot learns its steering behaviour directly from:
````
Sensor Feedback
      +
   Actions
      +
   Rewards
      ↓
 Learned Policy
````
Rather than explicitly programming every possible sensor condition and steering response, the desired behaviour emerges from the Q-learning process.

The resulting system demonstrates the integration of:

- ESP32 embedded programming
- FreeRTOS multitasking
- Infrared sensor processing
- Motor control
- Tabular Q-learning
- Exploration and exploitation
- Real-time reinforcement learning

## 📌 Description

This project implements a **tabular Q-learning algorithm** on an ESP32 to control a **5-sensor line-following robot**.

Instead of explicitly programming a fixed steering rule such as:

> "If the line is detected on the left, turn left."

the robot learns which motor action is most appropriate for each sensor condition through **reinforcement learning**.

The basic learning process is:

        IR Sensors
             │
             ▼
        Current State
             │
             ▼
      Select an Action
             │
             ▼
        Move the Robot
             │
             ▼
      Read New State
             │
             ▼
       Calculate Reward
             │
             ▼
       Update Q-Table
             │
             └──────────────► Repeat
# 🧠 Reinforcement Learning
1. State

The robot uses five infrared (IR) sensors to determine its position relative to the line.
The sensor arrangement is:
````
       L2    L1    C0    R1    R2
        │     │     │     │     │
        ▼     ▼     ▼     ▼     ▼
       [ ]   [ ]   [ ]   [ ]   [ ]
````
Each sensor produces a binary value:
1 → Black line detected
0 → No line detected

The five sensor outputs are combined into a 5-bit binary value.
Therefore, number of possible states = 2^5 = 32, range from 0 to 31
For example:

````
L2 L1 C0 R1 R2
 0  0  1  0  0
````
produces: 00100₂ = 4
Therefore, the sensor pattern can be used directly as the Q-table state index:

#🎮 Action

The robot has five possible motor actions:

Action	Description
0	Hard Left
1	Left
2	Centre
3	Right
4	Hard Right

The action determines the relative speed of the left and right motors.

Conceptually:
````
Hard Left     Left       Centre       Right      Hard Right
    ◄───────────◄──────────●───────────►────────────►
````
The five actions provide different levels of steering correction.

📊 Q-Table

The learned policy is stored in a Q-table:
````
float QT[32][5];
````
The table contains:
````
QT[state][action]
````
where:
state = current sensor state (0–31)
action = selected motor action (0–4)
QT[state][action] = estimated long-term reward of performing that action in that state

The table contains 32 states × 5 actions = 160 Q-values. Initially, all Q-values are set to zero:
````
              ACTION
           0    1    2    3    4
        ┌─────────────────────────
STATE 0 │ 0    0    0    0    0
STATE 1 │ 0    0    0    0    0
STATE 2 │ 0    0    0    0    0
 ...    │
STATE31 │ 0    0    0    0    0
        └─────────────────────────
````
This represents a robot with no prior knowledge about which action is best. As the robot interacts with the track, the Q-values are gradually updated. Eventually, the table represents the learned line-following policy.

# 🏆 Reward

The reward tells the robot whether its previous action was good or bad. The reward is determined from the new sensor state after an action has been executed. A state where the robot is centred over the line receives the highest reward. As the robot moves further away from the centre, the reward progressively decreases. Losing the line produces a strong negative reward.

Conceptually:
```
             Reward
               ▲
               │
          High │              ●
               │            Centre
               │
               │       ●           ●
               │
               │   ●                   ●
          Low  │
               │
               │
      Negative │ ●                     ●
               └──────────────────────────►
                  Left      Centre     Right
````
This reward structure teaches the robot that remaining near the centre of the line is desirable.

# 🔄 Q-Learning

After executing an action, the robot observes the new sensor state and updates the corresponding Q-value.
The standard Q-learning update equation is:
````
Q(s,a) = Q(s,a) + α [r + γ max Q(s',a') - Q(s,a)]
````
where:
s	Previous state
a	Selected action
r	Reward received
s'	New state after the action
a'	Possible action in the new state
α	Learning rate
γ	Discount factor

The important idea is that the robot does not only consider the immediate reward. It also considers the potential future reward of the new state.

Therefore, an action can be considered good even if its immediate reward is not the highest, provided that it leads to a state from which good future actions are possible.

# 🔍 Exploration vs. Exploitation

The robot uses an epsilon-greedy policy to balance exploration and exploitation.

## Exploration

With probability ε (epsilon), the robot selects a random action:
````
Random action
      │
      ▼
Try something new
      │
      ▼
Learn from the result
````
This prevents the robot from becoming trapped in a poor policy before it has sufficiently explored the available actions.

## Exploitation

With probability 1 - ε, the robot selects the action with the highest Q-value:
````
        QT[state]
            │
            ▼
     Find maximum Q
            │
            ▼
      Select best action
````
## Epsilon Decay

During training, epsilon gradually decreases. Conceptually:
````
ε
│\
│ \
│  \
│   \
│    \
│     \________
│
└──────────────────► Training Time
````
At the beginning: High ε → More exploration
Later: Low ε → More exploitation

This allows the robot to transition from trying different behaviours to using the policy it has learned.

# ⚙️ FreeRTOS Architecture

The ESP32 firmware is divided into separate FreeRTOS tasks.

Current tasks include:
````
┌──────────────────────────────┐
│          ESP32               │
│                              │
│  ┌────────────────────────┐  │
│  │ LED Status Task        │  │
│  └────────────────────────┘  │
│                              │
│  ┌────────────────────────┐  │
│  │ IR Sensor Task         │  │
│  └────────────────────────┘  │
│                              │
│  ┌────────────────────────┐  │
│  │ Q-Learning Task        │  │
│  └────────────────────────┘  │
│                              │
│  ┌────────────────────────┐  │
│  │ Motor Control Task     │  │
│  └────────────────────────┘  │
│                              │
└──────────────────────────────┘
````
The use of FreeRTOS allows different functions to operate as independent tasks rather than placing the entire control algorithm inside a single sequential loop. This makes it easier to add additional functions at a later stage without significantly disrupting the existing program structure.

For example, additional tasks could be added for:

Data logging
Serial communication
Bluetooth/Wi-Fi communication
Training statistics
Battery monitoring
User interface
Parameter adjustment
🧵 Task Isolation and Shared Data

Each FreeRTOS task has its own task stack, which provides separation between the task's local execution environment.

However, tasks can still access shared global variables.

For example:

             IR Sensor Task
                    │
                    ▼
              sensor_state
                    │
                    ▼
             Q-Learning Task
                    │
                    ▼
                action
                    │
                    ▼
             Motor Task

Because multiple tasks may access shared variables, synchronization mechanisms such as mutexes, semaphores, or critical sections may be required when there is a possibility of concurrent access.

In the current implementation, each important global variable is intended to have one task responsible for writing it, which reduces the risk of race conditions.

Nevertheless, this design should be reconsidered if future modifications allow multiple tasks to modify the same shared data.

## 🧮 ESP32 Dual-Core Processing

The ESP32 provides multiple CPU cores, which can potentially be used to distribute computational workload between FreeRTOS tasks.

For example:
````
             ESP32
          ┌───────────┐
          │           │
       Core 0       Core 1
          │           │
     ┌────┴────┐ ┌────┴────┐
     │ Sensors │ │ Q-Learn │
     │ Motor   │ │ Logging │
     │ Control │ │ Network │
     └─────────┘ └─────────┘
````
Task allocation should only be introduced when it provides a meaningful performance or responsiveness benefit. FreeRTOS already provides scheduling and task management, so simply adding more tasks or moving tasks between cores does not automatically improve performance.

# 🔁 Overall Learning Cycle

The complete reinforcement-learning cycle is:
````
        Read IR Sensors
               │
               ▼
        Determine State
               │
               ▼
       Select Action
     ┌─────────┴─────────┐
     │                   │
 Exploration         Exploitation
     │                   │
 Random Action       Best Q-value
     │                   │
     └─────────┬─────────┘
               ▼
        Control Motors
               │
               ▼
       Read New State
               │
               ▼
        Calculate Reward
               │
               ▼
        Update Q-Table
               │
               ▼
         Repeat Cycle
````
Over many interactions with the track, the Q-table gradually converges toward a useful line-following policy.
