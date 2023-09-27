---
layout: posts
title:  "The RL Framework: The Solution Part 1"
tagline: "Classical solutions to reinforcement learning problems in discrete space: \
**Monte Carlo methods**"
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

When we talk about the **solution** to a reinforcement learning (RL) problem, we are looking for a **policy** **$\pi$**, which is a mapping from **states $\mathcal{S}$** to **actions $\mathcal{A}$**. The policy defines the behavior of the agent. Our goal, is to find the optimal policy $\pi^*$, which maximizes the expected cumulative sum of rewards.

There are two types of policies:

  * A **Deterministic policy** is a mapping $\pi : \mathcal{S} \rightarrow \mathcal{A}$. For each state $s \in \mathcal{S}$, the policy yields a single action $a \in \mathcal{A}$, that the agent will take in state $s$.
  
  * A **Stochastic policy** is a mapping $\pi : \mathcal{S} \times \mathcal{A} \rightarrow [0,1]$. For each state $s \in \mathcal{S}$, and action $a \in \mathcal{A}$, it yields the probability $\pi(a \vert s)$ the agent will take action $a$ while in state $s$.


# Monte Carlo Methods
---
In the real world, we don't have any prior knowledge of the environment. Monte Carlo methods require only *experience* to learn the optimal policy, and from that experience, they can learn the optimal policy. To do that, the idea is to sample many many episodes (experiments), and learn valuable information about the state-value function $v_\pi$ and the action-value function $q_\pi$.
In the following discussion, we will always assume that we are in an episodic task, where the agent interacts with the environment for a finite number of time steps, and then the episode terminates. First we consider the *prediction problem* (the computation of $v_\pi$ and $q_\pi$ for some **fixed** policy $\pi$), then how to improve the policy, and finaly the *control problem* (the computation of the optimal policy $\pi^*$).

## MC - Prediction
---
Lets assume we have a given policy $\pi$. Recall that the value of the policy each state $s \in \mathcal{S}$ is the *expected return* which is the expected cumulative future discounted sum of rewards. To estimate it from experience, we can simply average the return observed after many visits to that state. The average will converge to the expected value by the law of large numbers. A good initial guess of the policy $\pi$ can be the *equiprobable random policy*, where the agent selects each action with equal probability. After many episodes, we consalidate the results to arrive at a better policy.

> **_Prediction Problem_**: Given a policy $\pi$, how might the agent estimat the value function $v_\pi$?

To estimate the value function $v_\pi$, we summarize the returns observed for each state $s \in mathcal{S}$ and action $a \in \mathcal{A}$, in a table known as a **$Q$-table**. If run a very large number of episodes, we will most likely gather enough information for every state-action pair $(s,a)$. 

A simple example can be shown here:
![simple-MC-example](/assets/images/Reinforcement-Learning/simple-MC-example.png)\
*An example: in every state (1, 2, 3, 4) with rewards (-1, -1, -1, +10) respectively, the agent can select an action right/left/up/down with equal probability.*

After running many episodes, we end up with a *$Q(s,a)$ table* that looks like this:
![simple-MC-example-Q-table](/assets/images/Reinforcement-Learning/simple-MC-example-Q-table.png)\
*The $Q(s,a)$ table after running many episodes.*

The table represents the expected return, if a agent starts at state $s$, selects action $a$, and then uses the policy to select all future actions.
If this sounds familiar, it's because it is! Each cell $Q(s,a)$ table calculates the *action-value function for the tested policy* $q_\pi(s,a)$, which is the expected return starting from state $s$, selecting action $a$, and then following policy $\pi$.

## Pseudo Code
---
There are two ways to estimate the value function $v_\pi$, depending on how you decide to treat the case where - *in a single episode* - the same action is selected from the same state multiple times:

  * **First-visit MC method**: The value of a state is the average of the returns following **_only first_** visits to that state.
  * **Every-visit MC method**: The value of a state is the average of **_all_** the returns following visits to that state.

Here I will consider first visit MC prediction:
![first-visit-MC-prediction](/assets/images/Reinforcement-Learning/first-visit-MC-prediction.png)

where:
* $N(s,a)$ is the number of times state-action pair $(s,a)$ was first visited (we count only the first visit in the episode).
* $return \text{_} sum$ - sum of the rewards obtained after first visit to the state-action pair $(s,a)$.
* $Q(s,a)$ - the Q table, sum of the rewards obtained after first visit to the state-action pair $(s,a)$.

So, after calculating the $Q$-table, how can we use it to improve our policy, and find the optimal policy $\pi^*$?

## MC Control, Greedy Policies and $\epsilon$-Greedy Policies
---
So, MC methods allows us to take a policy $\pi$, use it to interact with the environment for many episodes, and thenuse the results to estimate the action-calue function $q_\pi$. with a $Q$-table. 
Once we calculated the $Q$-table, we will use it to improve our policy. One way we can do that, is to select the action that has the highest value of each state in the $Q$-table, and use that as our policy $\pi'$. We will updated our policy to the new policyt $\pi \rightarrow \pi'$ and then collect many episodes again using the new policy $\pi'$. We will repeat this process again and again, always selecting the policy with the action that maximaizes the return value for each state.

> This is called a **greedy policy**. The greedy policy is defined as: $\pi(s) = \underset{a}{\operatorname{argmax}} Q(s,a)$. This means, it always selects the action that maximizes the return value for each state.

However, the reality is more complex and the greedy policy is not always the optimal policy. For example, consider a case where the agent is in a state $s$, with only two possible actions: 

$$ 
a_1 = 
\begin{cases}
1, & p=0.5 \\
3, & p=0.5
\end{cases}
$$

and 

$$
a_2 = 
\begin{cases}
0, & p=0.95 \\
1000, & p=0.05
\end{cases}
$$

It is easy to see that a possible outcome of a greedy policy is to always select action $a_1$ becuase in the majority of episodes, $a_2$ will yield 0, and $a_1$ will yield a result greater than 0. However, at a certain probability, the expected return of $a_2$ is much higher than the expected return of $a_1$.If the greedy policy tells us to always select $a_1$ at state $s$, we will never find the optimal policy. This is a problem. To solve it, we must always allow the agent to continue exploring the environment, and not always select the greedy action.
Hence we introduce a new concept:
> The **$\epsilon$-greedy policy**: a policy $\pi$ is called $\epsilon$-greedy if for every state $s \in \mathcal{S}$, it selects a random action with probability $\epsilon \in [0,1]$, and selects the greedy action with probability $1-\epsilon$.
>
$$
a_2 \leftarrow 
\begin{cases}
1-\epsilon + {\epsilon}/{|\mathcal{A(s)}|}, & \text{if }a \text{ maximizes } Q(s,a) \\
{\epsilon}/{|\mathcal{A(s)}|}, & \text{ else}
\end{cases}
$$

This way we guarantee that with probability $\epsilon$ the agent selects an action *uniformaly* from a set of available actions. This way, we can avoid the problem of the greedy policy, and still have a chance to select the best action.

> The process of finding the optimal policy with MC methods is called **Monte Carlo Control**. The algorithm is as follows:
> * **Step 1 (policy evaluation)**: use some policy $\pi$ to to construct a $Q$-table.
> * **Step 2 (policy improvement)**: Improve the policy by changing it to be $\epsilon$-greedy with respect to the $Q$-table.
> Repeat steps 1 and 2 until the policy converges to the optimal policy $\pi^*$.

![MC-control](/assets/images/Reinforcement-Learning/MC-control.png)

We now have a method for finding an optimal policy $pi^*$. However, the algorithm above is not very efficient. We will need to update our policy many many times. Instead, we need to find more efficient ways to update our policy.  This is somewhay similiar finding a better learning rate in nueral networks. We will introduce two concepts that will help us do that:
* **Incremental Mean**: update the policy after each step.
* **Constant-alpha**: leverage the most recent experience more effectively.

### The Exploration-Exploitation Dilemma
![exploration-exploitation-dilemma](/assets/images/Reinforcement-Learning/exploration-exploitation.png)

In the previous section, we introduced the $\epsilon$-greedy policy. However, we didn't discuss how to choose the value of $\epsilon$. The value of $\epsilon$ is very important. If we set $\epsilon$ to a very small value, the agent will always select the greedy action, and will never explore the environment. If we set $\epsilon$ to a very large value, the agent will always select a random action, and will never learn the optimal policy. This is called the **exploration-exploitation dilemma**. We need to find the right balance between exploration and exploitation. There are many ways to do that, but the most common way is to use a **decaying $\epsilon$**. We start with a value of $\epsilon \leftarrow 1$ (which yeilds the equiprobable random policy), and then we decay it over time $\epsilon \leftarrow 0$. This way, the agent will explore the environment more in the beginning, and exploit the environment more towards the end. This is a good strategy because in the beginning, the agent doesn't know anything about the environment, and it needs to explore it to learn more about it. Towards the end, the agent already knows a lot about the environment, and it can exploit it to maximize the return. 

In order to guarentee that MC contol converges to the optimal policy $\pi^*$, we need to make sure that two conditions are met, which are referred to as the **GLIE conditions**:

> **Greedy in the Limit with Infinite Exploration (GLIE)**: 
> * every state-action pair $(s,a)$ is visited infinitely many times, and
> * the policy converges on a greedy policy with respect to the action-value function $q_\pi$.

These two conditions ensure that the agent will explore the environment enough to learn the optimal policy $\pi^*$, and gradually exploits more. This is satisfied if:
* $\epsilon_i > 0$ for all timesteps $i$.
* $\epsilon_i \rightarrow 0$ as $i \rightarrow \infty$.

In practice it is recommend to either use:
* Fixed $\epsilon_i$
* Letting $\epsilon_i$ decay to a small positive number, like 0.1

In our current solution to MC control, we collect a large number of episodes to build the Q-table. However, this is not very efficient. While decaying $\epsilon$ handles the exploration-expoltation dilemma, it doesn't solve the problem of the large number of episodes. 

> Can we find ways to update our policy more efficiently? 
The answer is yes, and we will discuss two methods to do that:
> * **Incremental Mean**: update the policy after each step.
> * **Constant-alpha**: leverage the most recent experience more effectively.


### Incremental Mean
Update the policy after each step. This way we don't need to wait for the end of the episode to update the policy, and it will converge faster. The incremental mean is defined as:

$$
Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \frac{1}{N}(G_t-Q(s_t, a_t))
$$

where:
* $Q$ is the current value of the state-action pair $(s,a)$.
* $N$ is the number of *first* visits to the state-action pair $(s,a)$.

The pseudocode for the incremental mean is as follows:

![incremental-mean](/assets/images/Reinforcement-Learning/incremental-mean.png)

**Step 1**: The policy $\pi$ is improved to be $\epsilon$-greedy with respect to $Q$. The agent uses $\pi$ to collect an episode.
**Step 2**: $N$ is updated to count the total number of first visits to each state-action pair $(s,a)$.
**Step 3**: The $Q$-table is updated to take into account the most recent information.

### Constant-alpha
Looking at the previous equaitons, we can see that the value of $Q(s,a)$ is updated by adding a fraction of the difference between the observed return $G_t$ and the current value $Q(s,a)$. 
We can review the last term in the equation as an error term:
$$
\delta_t = G_t - Q(s_t, a_t)
$$

Looking at the error term, we can multiply it by some constant $\alpha \in [0,1]$, instead of just the number of first visits $N$. This way, we can leverage the most recent experience more effectively. This is called the **constant-alpha** method. We can write the equation as follows:

$$
Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha(G_t-Q(s_t, a_t))
$$


The pseudocode is as follows:

![constant-alpha](/assets/images/Reinforcement-Learning/constant-alpha.png)

#### Setting the Value of $\alpha$
we will write the equation again:

$$
Q(s_t, a_t) \leftarrow (1-\alpha)Q(s_t, a_t) + \alpha G_t
$$

Setting the value of $\alpha$ determines how much the weight the error between $Q$ and $G$ recieves while updating the table. This resembles learning rate in neural networks. 

* If $\alpha=0$, the action-value is never updated.
* If $\alpha=1$, then the final value estimate for each action-value is always equal to the last observed return $G_t$.
  
When implement constant-$\alpha$ MC control, be careful.
Setting the value too close to 1, the policy will not converge to the optimal polict. Setting the value too close to 0, will result in the agent learning too slowly.


# The problem with MC methods
One of the problems with MC control methods as that they recuire the episode to end before updating the policy. This is not very efficient and can even be dangerous is real life secarios. For example: We cannot train self-driving cars this way, because the episode will end only when the car crashes, or reaches its goal. But instead we would like make the agent learn "on the fly" and correct the policy on the move. In the next section, we will discuss a new method called **Temporal Difference (TD) Learning**. TD methods are more efficient than MC methods, and they don't require the episode to end before updating the policy.

* [The RL frame work - the solution part 2: Temporal Difference (TD) Learning]({% post_url 2023-09-10-The-RL-Framework-The-Solution-Part-2-TL-Methods %})
