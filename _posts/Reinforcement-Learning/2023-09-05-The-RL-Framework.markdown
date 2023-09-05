---
layout: post
title:  "The RL Framework: The Problem"
date:   2023-09-05 12:47:0
usemathjax: true
---

# Summary

<img src="/assets/images/Reinforcement-Learning/2023-09-05-The-RL-Framework/highlevel_overview.png" width="800">

# The Setting, Revisited
---
* The reinforcement learning (RL) framework is charcterzied by an **agent** interacting with an **environment**.
* At each timestep, the agent receives an **observation** of the environment's **state** and selects an **action** in response. One time step later, the agent recieves a **reward** and a new **state**.
* All agents have the goal to maximize the **cumulative reward** they recieve over all time steps.

# Episodic vs Continuing Tasks
---
* A **task** is an instance of the reinforcement learning (RL) problem.
* **Continuing tasks** are tasks that continue forever, without end.
* **Episodic tasks** are tasks that terminate at some point.
  * In this case, we refer to a complete sequence of interaction, from start to finish, as an **episode**.
  * Episodic tasks come to an end whenever the agent reaches a **terminal state**.

# The Reward Hypothesis
---
* **Reward hypothesis**: All goals can be described by the maximization of expected cumulative reward.

# Cumulative Reward
---
* The **return at time step** $t$ is: $G_t = R_{t+1} + R_{t+2} + R_{t+3} + ... 
* The agent selects actions with the goal of maximizing the expected return.

# Discounted Return
---
* The **discounted return** at time step $$t$$ is: $$G_t = R_{t+1} + \gamma R_{t+2} + \gamma^2 R_{t+3} + ... $$
* The discount rate $$\gamma$$ is a constant that you set.
  * $$\gamma$$ must satisfy $$0 \leq \gamma \leq 1$$
  * If $$\gamma = 0$$, the agent only cares about maximizing immediate rewards.
  * If $$\gamma = 1$$, the return is not discounted.
  * For larger values of $$\gamma$$, the agent cares more about the distant future. Smaller values of $$\gamma$$ result in more extreme discounting, where in the limit $$\gamma \rightarrow 0$$, the agent only cares about maximizing immediate rewards.

# MDPs and One-Step Dynamics
---
* The **state space** $$\mathcal{S}$$ is the set of all (*nonterminal*) states.
* In episodic tasks we use $$\mathcal{S}^+$$ to denote the set of all states, including terminal states.
* The **action space** $$\mathcal{A}$$ is the set of all possible actions. $$\mathcal{A}(s)$$ is the set of actions available in state $$s \in \mathcal{S}$$.
* The **one step dynamics** of the environment defines the probability of the next state and reward, given the current state and action.
  * $$p(s',r|s,a) \doteq Pr\{S_{t+1}=s',R_{t+1}=r|S_{t}=s,A_{t}=a\}$$
  * $$\sum_{s' \in \mathcal{S}} \sum_{r \in \mathcal{R}} p(s',r|s,a) = 1$$
* A **(finite) Markov Desicition Process (MDP)** is defined by:
  * A finite set of states $$\mathcal{S}$$ (or $$\mathcal{S}^+$$ for episodic tasks)
  * A finite set of actions $$\mathcal{A}$$
  * The one-step dynamics of the environment $$p$$
  * The **discount rate** $$\gamma \in [0,1]$$