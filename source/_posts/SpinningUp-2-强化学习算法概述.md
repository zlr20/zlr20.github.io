---
title: SpinningUp(2)强化学习算法概述
date: 2020-11-24 11:45:37
mathjax: true
categories:
- 强化学习
tags:
- RL
---

了解了强化学习术语和符号含义等基础内容后，我们将关注当下强化学习前沿研究领域中的算法蓝图，同时对这些算法的细节部分做简要的描述。

<img src='http://i1.fuimg.com/728885/a20937176a7cedd0.png'>

<!--More-->

## 免模型学习 vs 有模型学习

RL算法中最重要的区分之一是智能体是否已知或可以学习环境模型。这里，环境模型指的是一个可以预测状态转换和实时奖励的函数。

拥有模型的主要好处在于，它允许智能体进行前瞻性规划，了解一系列可能选择的结果，并明确决定其选择方案。然后，智能体可以将预先规划的结果提炼成学习到的策略。这一方法与无模型的方法相比，可以大大提高样本效率。

这种方法的主要缺点是智能体通常不能获得ground-truth的环境模型。如果智能体想在这种情况下使用模型，则必须纯粹从经验中学习模型，这会带来一些挑战。最大的挑战就是，智能体探索出来的模型和真实模型之间存在误差，而这种误差会导致智能体在学习到的模型中表现很好，但在真实的环境中表现得不好（甚至很差）。基于模型的学习从根本上讲是非常困难的，即使你愿意花费大量的时间和计算力，最终的结果也可能达不到预期的效果。

相比较而言，免模型学习虽然放弃了有模型学习在样本效率方面的潜在收益，但是他们往往更加易于实现和调整。截止到目前，相对于有模型学习，免模型学习方法更受欢迎，得到更加了广泛的开发和测试。



## Model Free RL相关算法

Model Free RL中有两种学习策略的思路，一种是基于策略的，另一种是基于价值的。

**策略优化（Policy Optimization）** ：这族算法将策略显示表示为$\pi_{\theta}(a|s)$ 并直接对性能指标$J(\pi_{\theta})$ 进行梯度优化（指标一般是期望回报），或者间接地对性能指标的局部近似函数进行优化。这种优化基本都是 **同策略**（on-policy) 的，也就是说每一步更新只会用当前策略执行时采集到的数据，之前的数据全部放弃。另外，策略优化通常也要学习出价值函数 $V_{\phi}(s)$ ，作为 $V^{\pi}(s)$的近似，该函数是确定策略梯度时的一项。

基于策略优化的方法举例：

- [A2C / A3C](https://arxiv.org/abs/1602.01783), 通过梯度下降直接最大化性能指标。
- [PPO](https://arxiv.org/abs/1707.06347) , 不直接通过最大化性能指标更新，而是maximizing a *surrogate objective* function which gives a conservative estimate for how much $ J(\pi_{\theta})$ will change as a result of the update。

**Q-Learning** ：这个系列的算法学习最优行动值函数$Q^*(s,a)$的近似函数$Q_{\theta}(s,a)$。这族算法通常使用基于贝尔曼最优方程的目标函数。优化过程是 **异策略**（off-policy） 的，这意味着每次更新可以使用任意时间点的训练数据，不管获取数据时智能体选择如何探索环境。

智能体的行动由$a(s) = \arg \max_a Q_{\theta}(s,a)$给出。

基于 Q-Learning 的方法举例：

- [DQN](https://www.cs.toronto.edu/~vmnih/docs/dqn.pdf), 一个让深度强化学习与强化学习结合的经典方法。
- [C51](https://arxiv.org/abs/1707.06887), 学习关于回报的分布函数，其期望是$Q^*$。

**策略优化和 Q-learning 的融合方法** ：策略优化和 Q-learning 并不是不能兼容的（在某些场景下，它们两者是 [等价的](https://arxiv.org/abs/1704.06440) ），并且存在很多介于两种极端之间的算法。这个范围的算法能够很好的平衡好两者之间的优点和缺点，比如说：

- [DDPG](https://arxiv.org/abs/1509.02971) 是一种同时学习确定性策略和 Q 函数的算法
- [SAC](https://arxiv.org/abs/1801.01290) 是一种变体，它使用随机策略、熵正则化和一些其它技巧来稳定学习，同时在 benchmarks 上获得比 DDPG 更高的分数。



## Model Based RL相关算法

与免模型的RL不同，对于基于模型的RL而言，不容易将算法分类，因为不同算法有许多不同的模型使用方式。

**纯规划** ：这是最基础的方法，也不显式地表示策略，而是纯使用规划方法来选择行动，例如模型预测控制(MPC)。在模型预测控制中，智能体每次观察环境的时候，都会计算得到一个对于当前模型最优的规划，这里的规划指的是未来一个固定时间段内，智能体会采取的所有行动（通过学习值函数，规划算法可能会考虑到超出范围的未来奖励）。智能体先执行规划的第一个行动，然后立即舍弃规划的剩余部分。

- [MBMF](https://sites.google.com/view/mbmf) 在一些深度强化学习的标准基准任务上，基于学习到的环境模型进行模型预测控制。

**专家迭代**（Expert Iteration） ：纯规划的后来之作，使用、学习策略的显式表示形式$\pi_{\theta}(a|s)$ 。智能体在模型中应用了一种规划算法，类似蒙特卡洛树搜索(Monte Carlo Tree Search)，通过对当前策略进行采样生成规划的候选行为。这种算法得到的行动比策略本身生成的要好，所以相对于策略来说，它是“专家”。随后更新策略，以产生更类似于规划算法输出的行动。

- [ExIt](https://arxiv.org/abs/1705.08439) 算法用这种算法训练深层神经网络来玩 Hex。
- [AlphaZero](https://arxiv.org/abs/1712.01815) 是这种方法的另一个例子。

**为免模型方法进行数据增强** ：仍使用免模型算法来训练策略或者 Q 函数，要么更新智能体的时候，用构造出的假数据来增加真实经验；要么更极端，更新的时候只使用构造的假数据。

- [MBVE](https://arxiv.org/abs/1803.00101) 用假数据增加真实经验
- [World Models](https://worldmodels.github.io/) 全部用假数据来训练智能体，所以被称为“training in the dream”。

**Embedding Planning Loops into Policies.** ：这种方法直接把规划程序作为策略的子程序，这样在基于任何免模型算法训练策略输出的时候，整个规划就变成了策略的附属信息。这个框架最核心的概念就是，策略可以学习到如何以及何时使用规划。这使得模型偏差不再是问题，因为如果模型在某些状态下不利于规划，那么策略可以简单地学会忽略它。

- 参见 [I2A](https://arxiv.org/abs/1707.06203)

