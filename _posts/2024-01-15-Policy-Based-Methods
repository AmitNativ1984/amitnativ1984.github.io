---
layout: posts
title:  "Policy Based Methods"
tagline: "Hill Climbing, Cross-Entropy Method, and Policy Gradient"
date:   2024-01-15
categories: REINFORCEMENT-LEARNING
permalink: /:categories/:title
header:
  overlay_image: /assets/Reinforcement-Learning/Policy-Based-Methods/hill-climbing.png
  overlay_filter: 0.5
toc: true
created: true
classes: wide
---
# Policy-Based Methods
* With **value-based-methods**, the agents uses its experience with the environment to maintain an estimate of the optimal action-value function. The optimal policy is the obtained from the optimal action-value function estimate.
* **Policy-based methods** directly learn the optimal policy, without having to maintain a separate value function estimate. Policy based methods can learn either stochastic or deterministic policies, and can handle continuous action spaces.

# Policy Function Approximation
* In deep reinforcement learning, it is common to represent the policy with a neural network.
  * The network takes the environment state as **input**
  * If the environment has discreate actions, the **output** layer has a node for each possible action and contains the probability that the agent should select each possible action.
  * The weights in this neural network are initially set to random variables. Then, the agent updates the weights as it interacts with the environment.

# Black Box Optimization
Black box optimization is a class of optimization algorithms that do not require exact knowledge of the gradient of the function being optimized.  

## Hill Climbing and Steepest Ascent
**Hill climbing** is a simple algorithm that can be used to find the weights of a neural network of an optimal policy.The algorithm is as follows:
1. Initialize the neural network weights $\theta$ arbitrarily.
2. Collect an episode with $\theta$, and record the return $G$:
   1. $\theta_{best} \leftarrow \theta$
   2. $G_{best} \leftarrow G$
3. Slightly perturb the values of the current best estimate of weights $\theta_{best}$, to get a new set of weights $\theta_{new}$.
4. Collect an episode with $\theta_{new}$, and record the return $G_{new}$.
      1. If $G_{new} > G_{best}$: 
      2. $\theta_{best} \leftarrow \theta_{new}$
      3. $G_{best} \leftarrow G_{new}$.
   
In the algorithm above, the weights are updated by adding a small amount of noise. However, This way of updating the weights is not very efficient because we are taking random directions up the hill. 
## Steepest Ascent Hill Climbing
This is variation of hill climbing that chooses a small number of neighboring policies at each iteration, and chooses the best among them.

## Simulated Annealing
In this method we sample the weights around the current value $\theta$. The sampling can be random/gaussian or even uniform, and then we pick the weights with the best value. The sampling radius is large in the beginning, and decreases over time. This allows the algorithm to explore the space in the beginning, and then converge to a local maximum.

## Adaptive Noise Scaling
Same as the previous method. That is, decreases the search radius with each iteration when a new best policy is found. However, if a new best policy is not found, the search radius is increased.

## Cross-Entropy Method
The cross-entropy method is a black box optimization method that can be used to find the optimal weights of a neural network. The algorithm is as follows:
In this method we take the average of the top 20% (parameter) episodes, and average them to get a new set of weights.

## Evolution Strategies
In evolution strategies, we run a weighted average over the weights of the different episodes. The higher the episode return, the higher the weight of the weights of that episode.

   

# Policy Objective Functions:
* Goal: given polivy $\pi_\theta(s,a)$ with paraters $\theta$, find best $\theta$.
* But, how do we measure the quality of a policy $\pi_\theta$?
* In episodic tasks, we can use the **start value** : $J_1(\theta) = v_{\pi_\theta}(s_1)$ 
* In continuing environments, we can use the **average value** : $J_{avV}(\theta) = \sum_{s\in S} \mu_{\pi_\theta}(s)v_{\pi_\theta}(s)$

  Where $\mu_{\pi_\theta}(s)$ is the probability of being in state $s$ in the long run.

* The **average reward per time-step** is defined as: $J_{avR}(\theta) = \sum_{s\in S} \mu_{\pi_\theta}(s)\sum_{a\in A} \pi_\theta(s,a)R_s^a$

  Where $R_s^a$ is the expected reward for taking action $a$ in state $s$.

Policy based reinforcement learning is an **optimization** problem:
* Find $\theta$ that maximizes $J(\theta)$.
* Some approaches do not use gradient-based optimization, such as:
  * Hill Climbing.
  * 
  ![hill-climbing](/assets/images/Reinforcement-Learning/Policy-Beased-Methods/hill-climbing-demo.jpg)
* However, it is also possible to use gradient-based optimization, such as stochastic gradient **ascent**. It is beneficial for deep learning because it also allows us to use back propagation.

So, Let $J(\theta)$ be any policy objective function. 
The policy graident algorithms search for a local maximum in $(J\theta)$ by following the gradient of the policy with respect to its parameters $\theta$.

$$ 
\begin{aligned}
\Delta\theta=\alpha\nabla_\theta J(\theta)
\end{aligned}
$$

Where $\nabla_\theta J(\theta)$ is the **policy gradient**, and $\alpha$ is a step size.

We will assume the policy is represented by a neural network, and it is differentiable with respect to its parameters $\theta$. 

Since the policy can in fact be stochastic, the goal is to calculate the gradient of the expected return:

$$
\begin{aligned}
\nabla_\theta J(\theta) = \nabla_\theta \mathbb{E_\mu}[v_{\pi_\theta}(S)]
\end{aligned}
$$

We will use Monte-Carlo samples to compute this gradient.
To do this we will use the **likelihood ratio trick**, also know as the **reinforce trick**.

$$
\begin{aligned}
\nabla_\theta\mathbb{E}[R(S,A)] = \mathbb{E}[\nabla_\theta log \pi(A|S)R(S,A)].
\end{aligned}
$$

Now, every time we interact with the environment, we can sample the gradient of the log probability of the action taken, and multiply it by the future return of the episode. 

So, the stochastic policy-gradient update is then:

$$
\begin{aligned}
\theta_{t+1} = \theta_t +\alpha R_{t+1} \nabla_\theta log \pi_{\theta_t}(A_t|S_t)
\end{aligned}
$$

Intuitively, we increase probability of actions with higher rewards