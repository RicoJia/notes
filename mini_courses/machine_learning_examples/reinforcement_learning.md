========================================================================
## Survey
========================================================================
There are three types of machine learning algorithms: 
- Supervised:
    - Learns from labelled data to predict the outcome of a given problem, by minimizing the prediction loss e.g., MNIST.
    - Iteratively evaluate by using SVM, Decision Tree, K-nearest neighbors, Deep Learning.
- Unsupervised:
    - Finds commonalities and structure between uncategorized data.
    - K-means, apriori, or PCA. 
- Reinforcement Learning:
    - Goal is given an environment, action and state space, we want to optimize the total reward. So the goal is different from
    unsupervised learning. 
    - Over time, we want to learn a series of optimal actions. 
    - There are three types of reinforcement learning:
        - Model based, MDP
        - Model free:
            - Value based: We optimize value of state to predict, or state-action pair and control optimally 
                - sarsa
                - q learning
                - When the state or state-action space is small enough, we can use a table to represent it (aka tabular learning). But when the state space is large, we want to use function approximation methods.
            - Policy based: (high variance during training?)
                - Reinforce (monte carlo policy gradient)
                - Deterministic Policy Gradient (DPG)
                - This is more similar to how we think: to figure out a strategy, unlike value based methods, learning the values of all states & actions.
            - Actor-critic: evaluate both the policy and the value with statble convergence.
                - 

Deep learning helps reinforcement learning to learn features much faster (TODO, ?), which  allows us to learn larger and more complex goals.
A reinforcement learning problem can be abstracted as an Markov Decision Process. But sure, this Markov Decision Process can be very large.

### Applications 
In Advertisement, to maximize total revenue, we want to show viewers ads that they will most likely like. This is called "impression allocation", where an impression is an ad pop-up. Platforms use collaborative filtering or content based filtering to rank sellers using historical scores, which considers how many customers the sellers likely to get. The current algorithms consider the sellers' similarities to customers, but not discounts sellers make to attract more customers. So, reinforcement learning can be used to better evaluate seller's behavior. Also, reinforcement learning can be used to detect fradulent behaviors on a platform? 

On the other hand, sellers can use RL to allocate advertising budget across platforms. There's online bidding platforms that show a seller's ad if they win the bid. A seller can automatically bid based on the total reward of the combo of platforms they use.

## k-Armed Bandits 
K-armed bandit is a slot machine has its own lever, and is called a “bandit” because it can empty your wallet like a thief :)
- Each bandit has a probability to fail/succeed
- We have a fixed number of levers to pull in action
So, you need to constantly check exploration vs exploitation (epsilon-greedy) and evaluate $Q_k(s,a) = 1/k(r_1 + ... r_k)$, the long term total reward till $kth$ step, based on each step's action and state.
    - $Q_{k+1} = func(Q_k)$, so we can simply add reward onto the existing Q function. (Dynamic Programming)

- There's also UCB methods

Sample Implementation: https://github.com/ankonzoid/LearningX/blob/master/classical_RL/multiarmed_bandit/multiarmed_bandit.py

## Finite MDP
- "All info you need is the present" - that's Markov. I.e., $P(S_{t+1}|S_{t}) = P(S_{t+1}|S_{t}, S{t-1} ...)$
- "Things have a clear end (terminal state)" is an episode. Your education is episodic, life is continuous.
- A markov decision process is basically a state machine, but with probability marked on each outcome
    - And RL is built on top of rewards & transitional probability on a markov decision process
- First, we are interested in total rewards. $G_t = R_{t+1}+R_{t+2} ...$ But really, we want to have a diminishing effect (because it will accumulate)
    - $R_{t+1}$ and $R_{t}$ both widely exists
    - So, we want to discount the total rewards: $G_t=R_{t+1} + \gamma R_{t+2} ... = R_{t+1} + \gamma G_{t+1}$ 
- State transitions
    - given state and action, next state could be different 
    - reward could be **different** even when the next state is the same
    - You know: $P(s', r| s, a)$ for each current state and action
    - Definition
        $$P(S_{t+1}=s'|S_t=s, A_t=a) = \sum_{r} P(s', r|s, a)$$
    - Expected Rewards for a certain state, and action
        $$r(s, a) = E[R_{t+1}|S_t=s, A_t=a] = \sum rP(R_{t+1}=r|s, a) = \sum r \sum_{s'}P(R_{t+1}=r, s'|s, a)$$
    - Expected reward for a certain state, next state, and action (see the first one)
        $$r(s, a, s') = E[R_{t+1}|S_t=s, A_t=a, S_{t+1}=s'] = \sum rP(R_{t+1}=r|s, a, s') = \frac{\sum rP(s', r|s, a)}{P(s'|s,a)}$$
- A (stochastic) policy is a look up table of **being in state S, what's the probability of choosing action A**: $\pi (A|S)$
    - Value Function: **using this lookup table, what would be the total expected reward, starting from current state s**
        - State Value Function, (Estimate by recording average $R$ w.r.t each state visited)
            1. Set up
                $$ 
                v_{\pi}(s) = E_{\pi}[G_t|S_t=s] = E_{\pi}[R_{t+1} + \gamma R_{t+2} + ... |S_t=s]
                \\
                v_{\pi}(s') = E_{\pi}[G_{t+1}|S_{t+1}=s'] = E_{\pi}[R_{t+2} + \gamma R_{t+3} + ... |S_{t+1}=s']
                \\
                $$
            1. For a given pair of $(s, a, s', r)$, its "update" is 
                $$r + \gamma E_{\pi}[R_{t+2} + \gamma R_{t+3} + ... |S_{t+1}=s']$$
            1. But you could have multiple $(s', r)$, given $(s,a)$. When going for $s$ to $s'$, ingegrate over $r$, and $s$
                $$
                E_{\pi}[R_{t+1} + \gamma R_{t+2} + ... |S_t=s] = 
                \sum_{A} \pi_{A} (a|s) \sum_{r} \sum_{s'}P(s', r|s,a)(r + \gamma E_{\pi}[R_{t+2} + \gamma R_{t+3} + ... |S_{t+1}=s'])
                \\
                = \sum_{A} \pi_{A} (a|s) \sum_{r} \sum_{s'}P(s', r|s,a)(r + \gamma v_{\pi}(s')) 
                $$
            1. $v_{pi}$ is expected $G_t$ at $t$ when state is in $s$. I'm wondering if there's a guarantee that the value function will converge?
        - Action Value Function (Estimate by recording average $R$ w.r.t each states and actions visited)
            $$
            q_{\pi}(a, s) = E_{pi}[G_t|S_t=s, A_t=a] = E_{\pi}[R_{t+1} + \gamma R_{t+2} + ... |S_t=s, A_t=a]
            $$
        - Monte Carlo method: above estimation methods

    - Bellman Equation: TODO
    - Optimal Value: $v_{*}(s) = max_{A} q_{\pi*}(a,s)$, notice the `*` for optimality
        - You choose the max value of the next state + reward
            $$
            v_{*}(s) = max_{A} q_{\pi}(s,a) 
            \\
            =max_{A} \sum_{r} \sum_{s} P(s'r|s,a)[r + \gamma v_{*}(s')]
            $$
        - Q:
            $$
            q_{\pi*}(s) = \sum_{r} \sum_{s'} P(s', r|s, a)(r + \gamma max_{a'} q_{\pi *}(s', a'))
            $$

## Dynamic Programming
- Value Iteration
    1. Initialize $v(s)$ to 0 in all states
    1. policy evaluation : note we are trying to get the optimal value under the current policy?
        $$v_{k+1}(s) = max_{A} \sum_{r} \sum_{s'}P(s',r|s,a)[r + \gamma v_{*}(s')]$$
        - repeat until $$v_{k+1}(s)$$ doesn't change much
    1. policy improvement: $$\pi(s)$$ is the most rewarding action, like in policy iteration.
        - this is **deterministic**

-  General Policy Iteration
    1. Initialize $v(s)$ to 0 in all states
    1. Have a an initial policy $\pi (a|s)$
    1. for a given $\pi(a|s)$, policy evaluation (guess we are "evaluating" the policy by looking at the values?)
        1. Do one pass over these states, **note we are just evaluating the value of s, instead of achieving the optimal value**
            $$v_{k+1}(s) = \sum_{r} \sum_{s'}P(s',r|s,a)[r + \gamma v_{*}(s')]$$
            - $k$ is the number of iteration. 
        1. we keep sweeping the states, until the value function at each state doesn't chage
    1. Policy improvement (greedy)
        1. get action from the policy
        1. For each state $s$, Find the most rewarding action $a'$, $a' = argmax_{A} \sum_{s'r}P(s',r|s,a)[r + \gamma v_{*}(s')]$
        1. if $a != a'$ we need to go back to policy evaluation, with $\pi(s)=a'$
            - **note: action a could be a set of actions, as long as these actions are equally rewarding**

## Monte Carlo Methods
- TODO: confirm why it's called tabular methods?
Many times, we don't know the model of the world $P(s',r|s,a)$. So, for a given $(s,a)$we take average of next state, and rewards
Say we have the grid world. We want to have 
- $v(s)$, which could be all zeros
- Returns $s(s)$ for each state, which should be all zeros
- number of visits at each state $n(s)$ where every entry is zero
- policy $\pi(s) for each state

At this point, you don't know anything about the grid world. So, you do a bunch of experiments, in episode, **following $\pi$**

In Each episode, you have $s_1, a_1, r_1 ...$ till T. and Total Reward $G=0$. , 
Then, you go back in time, from time $T$ to $0$: Say now you are at state $s$,
1. $G_t=\gamma G+r_t$, 
1. $n(s)+=1$
1. $s(s)+=G_t$.
1. OPTIONAL: if you want to update policy $pi$:
    1. calculate $v(s) = s(s)/n(s)$, or $q(s,a) = s(s,a)/n(s,a)$
    1. you need optimal $(s,a, s')$ lookup
    1. $a_{policy} = \pi(a|s)$, get $s'=lookup(s,a_policy)$ if $v_(s')< v_(s)$, then $\pi(a|s)=a$

Then, you iterate through all episodes. 
When you are done, for each state, you have $v(s) = s(s)/n(s)$

First visit monte carlo vs Every visit monte carlo, [ref](https://ai.stackexchange.com/questions/10812/what-is-the-difference-between-first-visit-monte-carlo-and-every-visit-monte-car)
- In first visit monte carlo, **will skip updating the above steps if your $s$ has already been visited in the same episode**
- Every time monte carlo will finish the whole process


Of course, you can do the same with $q(s,a)$, for state, action pi. Then, you can do control on that more easily
1. When updating policy, you won't need the $(s,a, s')$. 
1. You still choose your policy greedily.

### On and Off Policy
**One problem is you can't gurantee that all states (and maybe actions) are explored**
So, you can choose to: 
1. choose random start states for each episode, following some policy
2. Have a stochastic policy of action for each state. **Note that in the above example, you don't change policy.**
    - the policy for selecting actions is called "behavior policy", policy for updating Q function is called "update policy"
    - on-policy is to keep updating the policy $\pi (a|s)$, i.e., the behavior policy and the update policy are the same
    - off policy has different behavior and update policies

### On-policy
on policy, similar to the above mc method. but different in that 
    - we evaluate q(s,a) on the go
    - update policy **Say we do it first time, with Q(s,a)**
1. Iteration
    - $q(s,a)$, which could be all zeros
    - Returns $s(s,a)$ for each state and action, which should be all zeros
    - number of visits at each state $n(s,a)$ where every entry is zero
    - policy $\pi$, epsilon-soft
    At this point, you don't know anything about the grid world. So, you do a bunch of experiments, in episode.
1. In one episode, you have $s_1, a_1, r_1 ...$ till T. and Total Reward $G=0$. , 
1. Then, you go back in time, from time $T$ to $0$: Say now you are at state $s$,
    1. $G_t=\gamma G+r_t$, 
    1. **If $s_t$ has not shown up yet:**
        1. $n(s,a)+=1$
        1. $q(s,a)+=G_t/n(s,a)$.
        1. get set of optimal actions  $A^*=argmax_a Q(s,a)$
        1. update policy of actions $a$ in $s_t$, $A(s_t)$:
            - Non-optimal actions $\epsilon / |A(s_t)|$
            - optimal actions: $1 - \epsilon + \epsilon/|A(s_t)|$

1. When you are done, for each state, you have $q(s,a)$, and policy $\pi(s,a)$
### Importance Sampling
If you want to get expectation, variance, etc. from a distribution that's hard to sample from, consider sampling from an easier distribution.
- But you need ratio between the two distributions, at each value. 

$$E(X) = \int xP(x)dx = \int x \frac{P(x)}{Q(x)}Q(x)$$
- people usually do $E(f(x)) = \int f(x)p(x)dx$

- However, if $Q(x)$ and $xP(x)$ distribution look too different, then $xP(x)/Q(x)$ is large at every $xP(x)$, 
so $var = E[X^2]-E[X]^2$ will become larger in those regions?



### Importance Sampling
- Need example: https://towardsdatascience.com/importance-sampling-introduction-e76b2c32e744
1. TODO: notes are in IPAD


### Off Policy
Have a behavior policy $b(a_i|s_i)$, and a target policy $\pi (a_i|s_i)$. Use the behavior policy for generating new episodes, but
target policy to update and return. 

- Video: https://www.youtube.com/watch?v=bpUszPiWM7o
1. TODO: notes are in IPAD

## TD Learning. 
In Monte Carlo method, we udpate using the final return of nth episode. 
    $$v_{n+1}(s) = v_{n}(s) + \alpha (G_n(s) - v{n}(s))$$
But in TD Learning, we can simplify this process by updating with the immediate reward, **which is online**
    $$v_{n+1}(s) = v_{n}(s) + \alpha (r + \gamma v_{n}(s') - v_{n}(s))$$
One step TD $TD(0)$ is the above:
1. Initialize V(s), have a given policy
1. Loop for each episode until the terminal condition is reached:
    1. Take an action given by policy, observe next state $s'$, and its $v(s')$, $r$
    1. update Value
        $$v_{n+1}(s) = v_{n}(s) + \alpha (r + \gamma v_{n}(s') - v_{n}(s))$$
    1. update $s -> s'$

TD Target is $R + \gamma v_{n+1}(s)$, to replace total discounted reward $G_n(s)$. TD error is $R + \gamma v_{n+1}(s) - v_n(s)$
**it's called bootstraping**, because you learn from another estimate.

1. on and off policy
    - SARSA updates its $Q$ using the actual action taken by $s'$, which corresponds to the behavior policy. So,
        it's **on-policy** 
    - Q Learning updates its $Q$ always chooses the most rewarding action of $s'$, which could be different from its actual actions.
        So its update policy is different than its behavior policy, so it's **off-policy**
    - Expected Policy can be either on or off policy. Update policy you use $\pi(a|s)$, could be, or different than your action policy
    $b(a|s)$

### SARSA
It's basically the above, just swap them with $Q(s,a)$
1. Initialize $Q(s,a)$, have a given policy $\pi$
1. Loop for each episode until the terminal condition is reached:
    1. Take an action $a$ based on epsilon-greedy policy (greedy means "choosing the most rewarding policy")
    1. observe next state $s'$, $r$
    1. Take action $a'$ based on policy, update its $Q(s',a')$ (From the sequence of things to update, S, A, R, S, A)
    1. Update Q value
        $$Q_{n+1}(s, a) = Q_{n}(s, a) + \alpha (r + \gamma Q_{n}(s', a') - Q_{n}(s, a))$$
    1. update $s -> s'$

### Q Learning (watkins)
Compared to SARSA, instead of using the policy, We take optimal action when updating Q. But we are still using the policy to find the next action
1. Initialize $Q(s,a)$ arbitrarily, have a given policy $\pi$
1. Loop for each episode until the terminal condition is reached:
    1, Take an action $a$ given by policy $\pi, observe next state $s'$, $r$
    1. Update Q value
        $$Q_{n+1}(s, a) = Q_{n}(s, a) + \alpha (r + \gamma max_{a'} Q_{n}(s', a') - Q_{n}(s, a))$$
    1. update $s -> s'$

### Expected SARSA:
Instead of updating with next actual action in SARSA
    $$Q_{n+1}(s, a) = Q_{n}(s, a) + \alpha (r' + \gamma Q_{n}(s', a') - Q_{n}(s, a))$$
We update with the expectated Q:
    $$Q_{n+1}(s, a) = Q_{n}(s, a) + \alpha (r' + \gamma \sum_{a'} \pi(a'|s')Q_{n}(s', a') - Q_{n}(s, a))$$

**Expected SARSA eliminates variance from random action selection**. Behaves better than SARSA.


### Maximization Bias and Double Q Learning
**Q(s'a') is seen as a random variable.**, then in Q Learning, because you always choose action that lead to current $max Q(s', a')$, there's always a difference between 
the $E[max Q(s',a')]$, and the $max EQ(s',a')$. This difference is called bias, and we can show the bias is positive:
1. $max(x)$ is a convex function. For convex function, there's Jensen's inequality, which states:
    $$E[f(x)] >= f(E[x])$$
1. So, $bias = E[max Q(s',a')] - max E[Q(s',a')] >= 0$
1. So, the expectation of the max among Q(s'a') is always larger than the max of the expectation among Q(s'a') 

So, there's double Q learning: TODO

[Implementation](https://rubikscode.net/2021/07/20/introduction-to-double-q-learning/): Have two $Q$ tables: $Q1(s,a)$, $Q2(s,a)$.
For each episode:
    1. $Q_total =Q_A+Q_B$, find the max action. 
    1. flip a coin. If updating $Q_A$:
        $$
        Q_{A,n}(s,a) = Q_{A,n}(s,a) + \alpha[R + \gamma Q_{B,n}(s_{n+1}, a) - Q_{A,n}(s,a)]
        $$
        - switch QA, QB if we want to update QB.
    1. $s_n\rightarrow s_{n+1}$

- TODO: 
    - How does this minimize bias?
    - why is SARSA online, QLearning offline?

### N step Bootstrapping
You basically use actions and updates to update $n$ steps afterwards. 
$$
G_{t:t+n} = \sum_{m=0}^{m=n-1}[\gamma^{m} * R_{t+m}] + \gamma^(n) v_{k}(s_{t+n})
v_{k+1}(s_{n}) = v_{k}(s_{n}) + \alpha (G_{t:t+n} - v_{k}(s_{t+n})
$$
- If you haven't got to finish the first $n$ steps yet, just stick to the policy, don't do anything.
- for off-policy learning, you want to consider importance sampling:
    - 

### Examples and More
- [Grid World](https://towardsdatascience.com/introduction-to-reinforcement-learning-rl-part-3-finite-markov-decision-processes-51e1f8d3ddb7)

- temporal difference learning: to update the value function, we have a small feedback loop with the future value:
    - We first play the game
    - Then we back up after we finish, update $V(s) = V(s) + \alpha (V(s') - V(s))$, 
        - wheren $\alpha$ is a step size parameter, positive integer