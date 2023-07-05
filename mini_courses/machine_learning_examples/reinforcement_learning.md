## k-Armed Bandits 
K-armed bandit is a slot machine has its own lever, and is called a “bandit” because it can empty your wallet like a thief :)
- Each bandit has a probability to fail/succeed
- We have a fixed number of levers to pull in action
So, you need to constantly check exploration vs exploitation (epsilon-greedy) and evaluate $Q_k(s,a) = 1/k(r_1 + ... r_k)$, the long term total reward till $kth$ step, based on each step's action and state.
    - $Q_{k+1} = func(Q_k)$, so we can simply add reward onto the existing Q function. (Dynamic Programming)

- There's also UCB methods

Sample Implementation: https://github.com/ankonzoid/LearningX/blob/master/classical_RL/multiarmed_bandit/multiarmed_bandit.py