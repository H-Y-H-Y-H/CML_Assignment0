# Robot Locomotion Controller and DNN Trainer

This project includes a Python-based robotic simulation environment designed for developing and testing locomotion controllers. Additionally, it features a module for training a Deep Neural Network (DNN) to predict new states (NS) based on sensory inputs (S) and actions (A) using collected SANS data.

## Project Structure

- `V000_env.py`: Contains the custom environment class for the robot simulation.
- `main.py`: Entry point for implementing the robot controller and initiating training of the DNN.
- `model.py`: Use Pytorch to design a DNN.
- `requirements.txt`: Lists all the dependencies required for the project.

## Prerequisites

- Python 3.9

## Installation

First, ensure you have Python 3.9 installed on your machine.

Next, clone this repository to your local machine and install the required packages using:

```bash
git clone <repository-url>
cd <repository-directory>
pip install -r requirements.txt
pip install torch torchvision torchaudio
```


## Requirements:
### 1. Designing the Controller
Ensure your controller uses the methods provided in V000_env.py to interact with the simulation environment.
Develop a controller that enables the robot to move forward. The controller should implement a mechanism to 
apply actions with a Gaussian noise of standard deviation 
s=0.1. This is essential to simulate realistic actuation where perfect precision isn't achievable.

### 2. Collect SANS Data
Using the designed controller, run the robot simulation to collect data for training. The data should be structured as follows:

SANS: V000_env.py line 164

S (State): The current state of the robot, which may include positions, velocities, and other relevant sensory data.\
A (Action): The action taken by the controller, post-noise application.\
NS (New State): The state of the robot after the action is applied.\
Data Collection: Store the sequence of S, A, NS tuples during each simulation run.

### 3. Design a Deep Neural Network (DNN) Model
Create a DNN model in model.py that will learn to predict the next state (NS) based on the current state (S) and the action (A) applied.

Inputs: Current state (S) and action (A).\
Outputs: Predicted next state (NS).\
Architecture: Define an appropriate neural network architecture capable of capturing the dynamics of the robot's movements.


### 4. Train the DNN
Develop a training routine for the DNN model using the collected SANS data.

Training Data: Use the SANS data collected from the simulations.\
Objective: Minimize the difference between the predicted next state and the actual next state observed in the data.\
Evaluation: Assess the model's performance based on loss metrics with a validation set to ensure generalization.