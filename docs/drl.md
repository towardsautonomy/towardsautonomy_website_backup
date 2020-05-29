---
permalink: /drl
---

# Deep Reinforcement Learning <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*  

---

# Continuous Control using DRL - Training Multiple AI Agents within Unity framework 

This project demonstrates the use of Deep Reinforcement Learning for training AI agents to perform continuous controls. I work with a couple of Unity ML-Agents environment to teach single-agent and multiple-agents in both independent and competitive settings. Deep Deterministic Policy Gradient (DDPG) policy optimization technique is used for all the tasks.

## Environments

### a. [Crawler](https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Examples.md#crawler) A creature with 4 arms and 4 forearms.

![](/docs/drl/img/crawler.gif)  
*Figure - Trained Crawler Agent* 

In this environment, The agents must move its body toward the goal direction without falling. The environment has 12 agents, each observes a state with length 129, and outputs a vector of action of size 12. This task is episodic.

### b. [Tennis](https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Examples.md#tennis) environment for training two agents to play against each other in a competitive manner.

![](/docs/drl/img/untrained_tennis.gif)  
*Figure - Untrained Agents in Tennis Environment* 

![](/docs/drl/img/tennis.gif)  
*Figure - Testing the Agents after training the agents for 1800 episodes*

In this environment, two agents control rackets to bounce a ball over a net. If an agent hits the ball over the net, it receives a reward of +0.1. If an agent lets a ball hit the ground or hits the ball out of bounds, it receives a reward of -0.01. Thus, the goal of each agent is to keep the ball in play.

The observation space consists of 8 variables corresponding to the position and velocity of the ball and racket. Each agent receives its own, local observation. Two continuous actions are available, corresponding to movement toward (or away from) the net, and jumping.

### c. [Reacher](https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Examples.md#reacher) environment for training Single and Multiple agents.

| Single Agent                         |  Multiple Agents                      |
|:------------------------------------:|:-------------------------------------:|
|![](/docs/drl/img/reacher_single_untrained_agent.gif)  | ![](/docs/drl/img/reacher_multi_untrained_agents.gif)   |  
| *Figure - Single Untrained Agent in Reacher Environment* | *Figure - Multiple Untrained Agents in Reacher Environment* |
|![](/docs/drl/img/reacher_single_agent.gif)  | ![](/docs/drl/img/reacher_multi_agents.gif)   |  
| *Figure - Testing on Single Agent after training a single agent for 1200 episodes* | *Figure - Testing on Multiple Agents after training a single agent for 1200 episodes* |
|![](/docs/drl/img/reacher_single_agent_trained_on_multi_agents.gif)  | ![](/docs/drl/img/reacher_multi_agents_trained_on_multi_agents.gif)   |  
| *Figure - Testing on Single Agent after training multiple agents for 50 episodes* | *Figure - Testing on Multiple Agents after training multiple agents for 50 episodes* |

In this environment, a double-jointed arm can move to target locations. A reward of +0.1 is provided for each step that the agent's hand is in the goal location. Thus, the goal of the agent is to maintain its position at the target location for as many time steps as possible.

The observation space consists of 33 variables corresponding to position, rotation, velocity, and angular velocities of the arm. Each action is a vector with four numbers, corresponding to torque applicable to two joints. Every entry in the action vector should be a number between -1 and 1.

Two separate versions of the Unity environment is used:  
- The first version contains a single agent.
- The second version contains 20 identical agents, each with its own copy of the environment.  

## [GitHub](https://github.com/towardsautonomy/drl_continuous_control_unity_mlagents)  

### For more details and implementation, visit the [GitHub](https://github.com/towardsautonomy/drl_continuous_control_unity_mlagents) page.

---

# Lunar Landing in OpenAI's Gym Environment

This project demonstrates training a DRL agent using Deep Q-Learning to perform Lunar Landing within the OpenAI Gym's environment.

![](/docs/drl/img/moonlanding_dqn.gif)

## [GitHub](https://github.com/towardsautonomy/moon_landing_gym_dqn)  

### For more details and implementation, visit the [GitHub](https://github.com/towardsautonomy/moon_landing_gym_dqn) page.

---

# Navigation of an Agent in Unity-ML Environment

![](/docs/drl/img/banana_navigation.gif)  

*Figure - A DQN agent is trained to navigate this environment and collect yellow bananas while avoiding blue bananas.*

In this project, an AI agent is trained to navigate (and collect bananas!) in a large, square world. 

A reward of +1 is provided for collecting a yellow banana, and a reward of -1 is provided for collecting a blue banana. Thus, the goal of your agent is to collect as many yellow bananas as possible while avoiding blue bananas.

The state space has 37 dimensions and contains the agent's velocity, along with ray-based perception of objects around the agent's forward direction. Given this information, the agent has to learn how to best select actions. Four discrete actions are available, corresponding to:

```
    0 - move forward.
    1 - move backward.
    2 - turn left.
    3 - turn right.
```

## [GitHub](https://github.com/towardsautonomy/unity_mlagents_banana_navigator)  

### For more details and implementation, visit the [GitHub](https://github.com/towardsautonomy/unity_mlagents_banana_navigator) page.