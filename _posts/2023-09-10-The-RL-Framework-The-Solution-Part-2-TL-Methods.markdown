---
layout: posts
title:  "The RL Framework: The Solution Part 2"
tagline: "Classical solutions to reinforcement learning problems in discrete space: \
**Temporal Difference Methods**"
date:   2023-09-10
categories: REINFORCEMENT-LEARNING
permalink: /:categories/:title
header:
  overlay_image: /assets/images/Reinforcement-Learning/Blackjack-4.webp
  overlay_filter: 0.5
toc: true
created: true
classes: wide
---

Monte Carlo (MC) control methods require us to complete the entire episode of interaction before updating the Q-table. Temporal Difference (TD) methods, on the other hand, update the Q-table after every step of interaction. This makes TD methods more efficient than MC methods. In this post, I will provide three solutions to the reinforcement learning problem in discrete space using TD methods.

## TD Control Methods
In general, TD control methods only require one step of interaction to update the Q-table. The methods below, all belong to a group called **sarsa methods**. The name sarsa comes from the fact that the update rule of these methods is based on the tuple $(S_t, A_t, R_{t+1}, S_{t+1}, A_{t+1})$.

In all of the methods, we always start with an initial Q-table. The agent selects an action based on the $\epsilon$-greedy policy. The agent then interacts with the environment and receives the reward $R_{t+1}$ and the next state $S_{t+1}$. The agent then selects the next action $A_{t+1}$ based on the specific algorithm. The agent then updates the Q-table based on the update rule of the specific algorithm. The agent then repeats the process until the Q-table converges.

## Sarsa
1. Start with an initial Q-table.
2. Select an action $A_t$ based on the ***$\epsilon$-greedy*** policy.
3. Interact with the environment and receive the reward $R_{t+1}$ and the next state $S_{t+1}$.
4. Select the next action $A_{t+1}$ based on the ***$\epsilon$-greedy*** policy.
5. Update the Q-table based on the update rule:
   
$$
    Q(S_t, A_t) \leftarrow Q(S_t, A_t) + \alpha(R_{t+1} + \gamma Q(S_{t+1}, A_{t+1}) - Q(S_t, A_t))
$$

6. Repeat steps 2-5 until the Q-table converges.


## Sarsamax (Q-learning)
This time, after reciving the reward $R_{t+1}$ and the next state $S_{t+1}$, we select the next action $A_{t+1}$ based on the greedy policy. In other words, we choose the action of the next state with the maximum expected return. 

**Note**: We still choose the current action $A_t$ based on the $\epsilon$-greedy policy!! it is only the ***next action*** $A_{t+1}$ that is chosen based on the greedy policy.

1. Start with an initial Q-table.
2. Select an action $A_t$ based on the ***$\epsilon$-greedy*** policy.
3. Interact with the environment and receive the reward $R_{t+1}$ and the next state $S_{t+1}$.
4. Select the next action $A_{t+1}$ based on the ***greedy*** policy.
5. Update the Q-table based on the update rule:
   
$$
  Q(S_t, A_t) \leftarrow Q(S_t, A_t) + \alpha(R_{t+1} + \gamma \max_a Q(S_{t+1}, a) - Q(S_t, A_t))
$$

6. Repeat steps 2-5 until the Q-table converges.

## Expected Sarsa
In this TD-control method, we update the state-action pair in the Q-table after every step of interaction. Instead of taking the reward of the greedy policy as in Q-learning, we use the expected return of all possible actions of the next state (the reward, times the probability of taking the action $a$ according to the policy):

**Note**: We still choose the current action $A_t$ based on the $\epsilon$-greedy policy!! it is only the ***next action*** $A_{t+1}$ that is chosen based on the greedy policy.


1. Start with an initial Q-table.
2. Select an action $A_t$ based on the ***$\epsilon$-greedy*** policy.
3. Interact with the environment and receive the reward $R_{t+1}$ and the next state $S_{t+1}$.
4. Select the next action $A_{t+1}$ based on the ***greedy*** policy.
5. Update the Q-table based on the update rule:
   
$$
  Q(S_t, A_t) \leftarrow Q(S_t, A_t) + \alpha(R_{t+1} + \gamma \sum_a \pi(a|S_{t+1}) Q(S_{t+1}, a) - Q(S_t, A_t))
$$

6. Repeat steps 2-5 until the Q-table converges.





