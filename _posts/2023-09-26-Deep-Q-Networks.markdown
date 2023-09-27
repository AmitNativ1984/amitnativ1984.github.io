---
layout: posts
title:  "Deep-Q-Networks"
tagline: "From TL-methods in continuous space to Deep-Q-Networks"
date:   2023-09-26
categories: REINFORCEMENT-LEARNING
permalink: /:categories/:title
header:
  overlay_image: /assets/images/Reinforcement-Learning/Playing-Atari-with-deep-reinforcement-learning-deepsense.ai’s-approach-1140x337.png
  overlay_filter: 0.5
toc: true
created: true
classes: wide
---

Up until now, we have been dealing with reinforcement learning problems in discrete space. However, for many real-world problems, the state space is continuous. In this post, I will provide a solution to the reinforcement learning problem in continuous space, using a method called Deep-Q-Networks (DQN).
I will start first by recapping the reinforcement learning problem in discrete space. Then, I will provide some methods that can help us solve problems in continous space using function approximations and methods used in discrete spaces. However, solving a Q-table with millions of states is not practical, so this will lead us to the DQN method.

## The reinforcement learning problem, a recap
In the reinforcement learning problem, we have an agent interacting with an environment. At each time step, the agent chooses an action $A_t$, and as a result, the agent receives a reward $R_{t+1}$ and the next state $S_{t+1}$ of the environment. 

![RL-problem](/assets/images/Reinforcement-Learning/2023-09-05-The-RL-Framework/highlevel_overview.png)

As I mentioned in the previous post, many reinforcement learning problems can be formulated as a Markov Decision Process (MDP). In this case, we have a finite set of states $\mathcal{S}$, a finite set of actions $\mathcal{A}$. The one-step dynamics of the environement are defined by the probability transitioning to the next state $S_{t+1}$ and receiving the reward $R_{t+1}$, given the current state $S_t$ and action $A_t$. This can be describes by a Makov Process, also known as a Markov Chain:

![Markov-Chain](/assets/images/Reinforcement-Learning/MDP-chain.webp)

The goal of our agent, for each time step, is to select the action, based on a policy that will maximize the total cumulative reward. However, because the enivronment is stochastic, the agent can only maximize the **expected** cumulative reward. This is known as the **reward hypothesis**.


## Some formal definitions

### The Markov Property
> ***Transition:** Moving from one state to another is called Transition.*
> 
> ***Transition Probability:** The probability that the agent will move from one state to another is called Transition Probability.*
> 
> *The **Markov Property*** states that:***The future is independent of the past, given the present.***

Mathematically, this can be described as:
$$
P(S_{t+1}|S_t) = P(S_{t+1}|S_1, S_2, ..., S_t)
$$
 functions of the state $s$. For example, if we have a state $s = (x_1, x_2)$, we can define the features as $\mathbf{x}(s) = 

$$
P = 
\begin{bmatrix}
    P_{11} & P_{12} & \dots  & P_{1n} \\
    P_{21} & P_{22} & \dots  & P_{2n} \\
    \vdots & \vdots & \ddots & \vdots \\
    P_{n1} & P_{n2} & \dots  & P_{nn}
\end{bmatrix}
$$

Where $P_{ij}$ is the probability of transitioning from state $i$ to state $j$.

### Reward and Returns
> ***Reward:** a numerical scalar value the agents receives on performing some **Action** at some state $s$*
> In reinforcement learning we care about **maximizing** the cumulative reward, instead of the reward the agent receives at a given state. This is known as the **return**.
> 
> ***Discount Factor $(\gamma \in [0,1])$**: Determines the **importance** given to the **immidiate rewards and future rewards**. Because the knowledge of the future is uncertain, we are not sure how the environment will change as we progress in time. This is why it is important to **discount** rewards further into the future.*

So, the discounted return is given by:

$$
G_t = R_{t+1} + \gamma R_{t+2} + \gamma^2 R_{t+3} + ... = \sum_{k=0}^{\infty} \gamma^k R_{t+k+1}
$$

### Policy Function and State-Value Function
For every time step $t$, the agent needs to choose an action, that will transition it to the next state which will maximize the expected return. 
> ***Policy Function $\pi(a|s)$:** the probability of choosing an action $a$, given the agent is at state $s$. The goal in reinforcement learning is to find the optimal policy $\pi^*$ that  maximizes the expected return*. 

$$\pi(a|s) = \mathbb{P}(A_t=a|S_t=s)$$

> ***State-Value Function $v_\pi(s)$:** the value of a state when the agent is following policy $\pi$ is the expected return if the agent starts at state $s$ and stochasticly follows policy $\pi$ thereafter until the episode ends*

$$
v_\pi(s) = \mathbb{E}_\pi[G_t|S_t=s] = \mathbb{E}_\pi[\sum_{k=0}^{\infty} \gamma^k R_{t+k+1}|S_t=s]
$$

### State-Action Value Function
In the end, the agent needs to choose the action, that will maximizes the expected return. Each state can have multiple actions, so we need to find the action that will maximize the expected return. 
> ***State-Action value or Q-Function $q_\pi(s,a)$:** The expected return, when the agent is at state $s$ and choose action $a$. When dealing with discrete space, this can be summarize in a Q-table*

$$
q_\pi(s,a) = \mathbb{E}_\pi[G_t|S_t=s, A_t=a] = \mathbb{E}_\pi[\sum_{k=0}^{\infty} \gamma^k R_{t+k+1}|S_t=s, A_t=a]
$$


### Bellman Equation
The Bellman equation is a fundamental equation in reinforcement learning. It describes the relationship between the value of a state and the value of its successor state, by decomposing the value of the state into the immediate reward and the discounted value of the successor state. 

> ***Bellman Equation for $v_\pi(s)$:** the value function of state $s$ given policy $\pi$, is the immediate expected reward $r$ from transition to the next state $s'$ by taking action $a$, plus the value of the successor state $v_\pi(s')$ with discount factor $\gamma$.*

$$
  v_\pi(s) = \sum_{a \in \mathcal{A}} \pi(a|s) \sum_{s' \in \mathcal{S}} \sum_{r \in \mathcal{R}} p(s',r|s,a) [r + \gamma v_\pi(s')]
$$

> ***Bellman Equation for $q_\pi(s,a)$:** the expected return when the agent is in state $s$ and chooses action $a$ is the immediate expected return $r$ of choosing action $a$, and the expected return of the next state $s'$ using policy $\pi$*

$$
  q_\pi(s,a) = \sum_{s' \in \mathcal{S}} \sum_{r \in \mathcal{R}} p(s',r|s,a) [r + \gamma \sum_{a' \in \mathcal{A}} \pi(a'|s') q_\pi(s',a')]
$$

# Solving the RL problem in continuous space
In the previous posts I described two methods to solve the reinforcement learning problem in discrete space. These methods are:
1. **Monte Carlo (MC) methods**: These methods require us to complete the entire episode of interaction before updating the Q-table.
2. **Temporal Difference (TD) methods**: These methods update the Q-table after every step of interaction.

These methods are handy, when we have a limited and descrete number of states. However, in many real-world problems, the state space is continuous, and these methods fail to work.

## Function Approximation
We are after the true state value function $v_\pi(s)$, and the true state-action value function $q_\pi(s,a)$. Both are continuous and smooth functions over the entire space. Capturing the true value function is impossible, so we need to approximate it. We can do this by choosing a parameterized that approximates the true value function. 

$$
\hat{v}(s, \mathbf{w}) \approx v_\pi(s) \\
\hat{q}(s, a, \mathbf{w}) \approx q_\pi(s,a)
$$

Where $\mathbf{w}$ is the parameter vector of the function approximator. Our goal is to find a set of parameters $\mathbf{w}$ that will minimize the error between the true value function and the approximated value function. 

Lets define our state $s$ as a vector of features $\mathbf{x}(s)$. These features form a base that spans our state space. We can then define our approximated value function as a linear combination of the features:

$$
\hat{v}(s, \mathbf{w}) = \mathbf{x}(s)^T \cdot \mathbf{w} = \sum_{j=1}^{n} \mathbf{x}_j(s) \mathbf{w}_j
$$

Where $\mathbf{w}$ is the parameter vector of the function approximator, and $\mathbf{x}_j(s)$ is the $j$-th feature of the state $s$. 

However, in reality, many times the function true value function is not linear. So, we can use a non-linear function approximator. To include non-linearity to the above linear combination, we can use a non-linear function:

$$
\hat{v}(s, \mathbf{w}) = f(\mathbf{x}(s)^T \cdot \mathbf{w})
$$

As $f$ is non-linear, we will optimize the parameters $\mathbf{w}$ using gradient descent.

$$
  \Delta \mathbf{w} = \alpha (v_\pi(s) - \hat{v}(s, \mathbf{w}))\nabla_\mathbf{w} \hat{v}(s, \mathbf{w})
$$

# Deep Q-Networks (DQN)
So, our goal is to use non-linear function approximators to approximate the true value function. Since we are using non-linear function approximators, we can use neural networks. This is where Deep Q-Networks (DQN) come in. DQN is a reinforcement learning method that uses a neural network as a function approximator to approximate the true value function. It was first introduced in a paper from deepmind called [Human-level control through deep reinforcement
learning](https://www.nature.com/articles/nature14236), and it was used to play Atari games.

Below is a design of the original network used in the paper, and its main components:

![DQN](/assets/images/Reinforcement-Learning/dqn.png)


**input:** The input to the network is the state vector $s$. In the paper, the authors used a 84x84x4 image. The 4 images are the last 4 frames of the game. This is because the agent needs to know the velocity of the ball, and the direction it is moving in. More generally, many times there is important corrlations between the current state and the previous states.

**The network:** The input is passed through a series of convolutional layers, followed by fully connected layers. Relu activation functions are used.

**Output Layer:** The output layer is a fully connected layer with a linear activation function. The output of the output layer is the Q-values for each action. Unlike traditional reinforcement learning methods where only one Q value is updated every time, here the network outputs all the Q-values for all the actions at the same time.

However, there are some problems with this network, and it may not converge to the optimal policy without additional twicks. There are two main reasons for this problem. The first is because the network is learning from its own predictions (as result of taking predicted actions). This means the training data is dependant in current state predictions. This can lead to a feedback loop, where the network will diverge. Another problem is that there is a high correlation between the current state and the previous states. This can lead to the network overfitting to the current state, and not learning the optimal policy. The authors of the paper proposed two solutions to these problems:

> ***Fixed Q-Targets**: to break the dependancy of between ouput (actions) and input (the next state is a result of action taken), we use two networks. The first network is the one we use to **predict** the Q-values. The second network is the one we use to **update** the Q-values. The second network is a copy of the first network, but its parameters are updated less frequently. This helps to break the feedback loop, and helps the network to converge.*

> ***Experience Replay:** Instead of learning from the current state, the network learns from a random sample of previous states. This helps to break the correlation between the current state and the previous states. In practice, tuples of $(S_t, A_t, R_{t+1}, S_{t+1})$ are stored in a circular buffer. While training, we occasionally sample a batch of tuples from the buffer, and use them to train the network.*

### Deep Q-Networks Algorithm
1. Initialize replay memory $D$ to capacity $N$
2. Initialize action-value function $Q$ with random weights $\mathbf{w}$
3. *(Fixed Q-target)* initialize target action-value function $\hat{Q}$ with same weights $\mathbf{w}^- = \mathbf{w}$
4. For episode $e = 1$ to $M$ do:
    1. Initialize input x_1$.
    2. Preprocess initial state: $S_1 \leftarrow \phi(x_1)$
    3. For $t = 1, T$ do:
        1. Choose action $a_t$ from the set of possible actions, using an $\epsilon-greedy$ policy.
        2. Execute action $a_t$ in emulator and observe reward $r_t$ and next input $x_{t+1}$
        3. Prepare next state $S_{t+1} \leftarrow \phi(x_{t-2}, x{t-1}, x_t, x_{t+1})$
        4. Store experience tuple $(S,A,R,S')$ in replay memory $D$.
        5. Sample random minibatch of transitions $(\phi_j, a_j, r_j, \phi_{j+1})$ from $D$
        6. $S \leftarrow S'$

        7. Obtain random minibatch of tuples $(s_j, a_j, r_j, s_{j+1})$ from $D$
        8. Set target $y_j = \begin{cases} r_j & \text{for terminal } s_{j+1} \\ r_j + \gamma max_{a'} \hat{q}(s_{j+1}, a; \mathbf{w}^-) & \text{for non-terminal } s_{j+1} \end{cases}$
        9. Perform a gradient descent step on $(y_j - q(s_j, a_j; \mathbf{w}))^2$ with respect to the network parameters $\mathbf{w}$: $\Delta \mathbf{w} = \alpha (y_j - \hat{q}(s_j, a_j; \mathbf{w})) \nabla_\mathbf{w} q(s_j, a_j; \mathbf{w})$
        10.  Every $C$ steps reset $\hat{Q} = Q$
    4. End For
5. End For


### Deep Q-Networks Algorithm improvements
Several improvments to the original Deep Q-Learning algorithms have been suggested. These improvements are:
1. **Double Deep Q-Learning (DDQN)** Double DQN is an improvment to the original DQN algorithm. The idea is to decouple the action selection from the action evaluation. This is done by using the online network to select the action, and the target network to evaluate the action. This helps to reduce the overestimation of the Q-values.
   
2. **Prioritized Experience Replay**: based on the idea that the agent can learn more effectively from some transitions than from others, and the more important transitions should be sampled with higher probability. Instead of sampling the replay buffer uniformly, we sample the replay buffer based on the TD-error and based on the frequency of events (improve sampling of less frequent events). 
3. **Dueling DQN**: The idea is to separate the network into two streams. One stream estimates the state value function $v(s)$, and the other stream estimates the advantage function $a(s,a)$. The two streams are then combined to estimate the Q-values. This helps to generalize learning across actions without imposing any change to the underlying reinforcement learning algorithm. 
   
![Dueling-DQN](/assets/images/Reinforcement-Learning/dueling-dqn.png)