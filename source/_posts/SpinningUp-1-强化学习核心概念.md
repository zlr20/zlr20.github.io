---
title: SpinningUp(1)强化学习核心概念
date: 2020-11-23 10:30:38
mathjax: true
categories:
- 强化学习
tags:
- RL
---

**强化学习**(Reinforcement Learning, RL)的主要特征是**智能体**(Agent)和**环境**(Environment)。

<img src='http://i1.fuimg.com/728885/a9d3f1a3998f9808.png'>

环境是智能体存在并与之交互的世界。在交互的每个步骤中，智能体都会获得对世界状态的观察（也许只是部分可观的），然后决定要采取的行动。当智能体执行动作后，环境会产生变化；环境也可能会自发变化。

智能体还从环境中感知到**奖励**(reward)信号，一个表明当前环境状态好坏的标量值。智能体的目标是最大化其累积奖励，也就是**回报**(return)。强化学习就是智能体通过与环境交互、学习行为策略从而完成这一目标的过程。

<!-- More -->

为了便于后面的学习，我们介绍一些术语：

- 状态和观察(states and observations)
- 动作空间(action spaces)
- 策略(policies)
- 轨迹(trajectories)
- 不同的回报公式(formulations of return)
- 强化学习优化问题(the RL optimization problem)
- 值函数(value functions)



## 1. 状态和观察 States&Observations

一个**状态**$s$是一个关于环境状态的完整描述，环境中除了状态以外没有更多的信息。**观察**$o$是对于一个状态的部分描述，可能会漏掉一些信息。

在深度强化学习中，我们一般用实数向量、矩阵或者更高阶的张量表示状态和观察。比如说，视觉上的**观察**可以用RGB矩阵的方式表示其像素值；机器人的**状态** 可以通过关节角度和速度来表示。

如果智能体观察到环境的全部状态，我们通常说环境是**完全可观**(fully observed)的。如果智能体只能观察到状态中的一部分信息，我们称之为**部分可观**(partially observed)。举个例子，Atari游戏或自动驾驶中的一帧画面，肯定是不能完整涵盖所有状态信息的，至少无法由一帧画面判定运动信息，因此这个场景就是部分可观的。解决部分可观环境中的决策问题，对人来说都非常困难，对于智能体来说，更是研究的热点与难点。

另外，值得注意的是，不同文献中，$s$和$o$经常出现混用，因为强化学习理论上，动作是基于状态而给出的，但是实际上，我们只能获取环境的观察值，而不能知道环境的全部状态。但不必为这个问题纠结，只要把握住状态和观察的核心概念与差别，就不会出现理解上的问题。



## 2.动作空间 Action Spaces

不同的环境允许不同类型的动作。所有有效动作的集合称之为 **动作空间**。有些环境，比如说 Atari 游戏和围棋，属于**离散动作空间**(discrete action spaces)，这种情况下智能体只能采取有限的动作。其他的一些环境，比如智能体在物理世界中控制机器人，属于**连续动作空间**（continuous action spaces)。在连续动作空间中，动作是实数向量。

这种区别对于深度强化学习来说，影响深远。有些种类的算法只能直接用在某些案例上，如果需要用在别的地方，可能就需要大量重写代码。



## 3. 策略 Policies

**策略**是智能体用于决定下一步执行什么行动的规则。

策略可以是确定性的，一般表示为：$a_t = \mu(s_t)$；也可以是随机的，一般表示为一个条件分布：$a_t \sim \pi(\cdot | s_t)$。

因为策略本质上就是智能体的大脑，所以很多时候“策略”和“智能体”这两个名词经常互换，例如我们会说：“策略的目的是最大化奖励”。

在深度强化学习中，我们处理的是参数化的策略（parameterized policies），这些策略的输出，依赖于一系列计算函数，而这些函数又依赖于参数（例如神经网络的权重和误差），所以我们可以通过一些优化算法改变智能体的的行为。

我们经常把这些策略的参数写作$\theta$或者$\phi$，然后把它写在策略的下标上来强调两者的联系，如$a_t = \mu_\theta(s_t)$；$a_t \sim \pi_\theta(\cdot | s_t)$。

**确定性策略**(Deterministic Policies)：

这是一个使用`PyTorch`构建连续动作空间+确定性策略的代码片段：

```python
pi_net = torch.nn.Sequential(
              nn.Linear(obs_dim, 64),
              nn.Tanh(),
              nn.Linear(64, 64),
              nn.Tanh(),
              nn.Linear(64, act_dim)
            )
```

这将构建一个多层感知器（MLP）网络，其中包含两个大小为64的隐藏层和激活函数。如果obs是包含一batch的观测值的Numpy数组，则pi_net可用于获取一batch的动作输出，如下所示：

```python
obs_tensor = torch.as_tensor(obs, dtype=torch.float32)
actions = pi_net(obs_tensor)
```

**随机性策略**(Stochastic Policies)：

Deep-RL中最常见的两种随机策略是分类型策略(categorical policies)和对角高斯策略(diagonal Gaussian policies)。分类型策略适用于离散操作空间，而对角线高斯策略适用于连续操作空间。

训练和使用随机策略的时候有两个重要的计算过程：

- 从策略中采样行动$a$，用在轨迹采集
- 计算特定行动的似然(likelihoods) $\log \pi_\theta(a|s)$，这与策略梯度相关

下面具体看一下：

**分类型策略**就像是一个离散动作空间的分类器(classifier)：输入是观察，接着通过一些卷积层、全连接层等，至于具体是哪些取决于输入的类型，最后一个线性层输出每个行动的logit数值，最后用一个Soft-max层把logit值转换为可能性。[YouTube: logit and softmax in deep learning](https://www.youtube.com/watch?v=Qn4Fme1fK-M)

- 采样：给定每个动作的概率，像`PyTorch`和`Tensorflow`这样的框架都有内置的采样工具。参考：[PyTorch中使用log-odds或prob数组采样](https://pytorch.org/docs/stable/distributions.html#categorical)和[TF中使用log-odds或prob数组采样](https://www.tensorflow.org/versions/r1.15/api_docs/python/tf/distributions/Categorical)
- 似然：Soft-max层输出不同动作的概率，表示为$P_\theta(s)$。它是一个向量，但是由于有多个动作，因此我们可以将动作视为向量的索引，即$a=0,1,2,..$。然后可以通过索引向量来获得动作的对数似然率，即$\log \pi_\theta(a|s) = \log(P_\theta(s))[a]$。

多元高斯分布由均值向量和协方差矩阵来描述。对角高斯分布是一种特殊情况，其中协方差矩阵是一个对角矩阵。因此我们可以用向量表示它。

**对角高斯策略**由一个神经网络表示，该神经网络将观测值映射为动作的均值$\mu_\theta(s)$（向量）。协方差矩阵有两种不同的表示方式，第一种方法是用一个对数标准差$\log \sigma$（向量），它不是状态的函数，而是独立参数；第二种方法是也由神经网络将对数标准差映射出来，即$\log \sigma_\theta(s)$。注意，在这两种情况下，我们都输出对数标准差而不是直接输出标准差的原因是标准差必须为非负数，而加入$\log$后则可以在整个$(-\infty,+\infty)$空间上映射，而且可以通过取e幂获得标准差，因此这种表示方式不会有任何损失。

- 采样：您可以构建标准的概率分布对象，例如[torch.distributions.Normal](https://pytorch.org/docs/stable/distributions.html#normal) 或[tf.distributions.Normal](https://www.tensorflow.org/versions/r1.15/api_docs/python/tf/distributions/Normal)并使用这些对象的sample方法生成样本。

- 似然：一个 $k$ 维行动 $a$ 基于均值为$\mu = \mu_\theta(s)$，标准差为$\sigma = \sigma_\theta(s)$ 的对角高斯的对数似然是

  ![](https://spinningup.openai.com/en/latest/_images/math/26f82323a4055444b30fa791238ec90913a12d7b.svg)



## 4. 轨迹 Trajectories

运动轨迹$\tau$指的是状态和行动的序列，即$\tau = (s_0, a_0, s_1, a_1, ...)。

第一个状态$s_0$是从**初始状态分布**中随机采样的：$s_0 \sim \rho_0(\cdot)$。

状态转换是由环境的“自然法则”确定的，并且只依赖于最近的行动$a_t$，即具有**马尔可夫性**。

状态转移方程可以是确定性的：$s_{t+1} = f(s_t, a_t)$；也可以是随机的：$s_{t+1} \sim P(\cdot|s_t, a_t)$。

**Trajectories**在文献中常常也被称作 **episodes** 或者 **rollouts**，它们是等价的。



## 5. 奖励和回报 Reward&Return

强化学习中，奖励函数$R$非常重要。它由当前状态、已经执行的行动和下一步的状态共同决定：$r_t = R(s_t, a_t, s_{t+1})$。有时候这个公式会被简化到只依赖当前的状态： $r_t = R(s_t)$，或者状态-行动对：$r_t = R(s_t,a_t)$。

智能体的目标是最大化行动轨迹的累计奖励（cumulative reward over a trajectory），表示为 $R(\tau)$，有的文献中成为回报G。

一种回报形式是有限长度的无折扣回报（finite-horizon undiscounted return），即在固定的步骤窗口中获得的奖励之和：$R(\tau) = \sum_{t=0}^T r_t$。

另一种回报形式是无限长度的有折扣回报（infinite-horizon discounted return），它依然是智能体获得的所有奖励的总和，但对将来获得的收益进行了数值上的折扣：$R(\tau) = \sum_{t=0}^{\infty} \gamma^t r_t$。$\gamma \in (0,1)$被称作折扣因子。

为什么我们需要折扣因子呢？我们不是只想获得所有奖励吗？是这样的，但是折扣因子在直观上和数学上都很方便。从直觉上讲：当下的收益比之后的收益要来的更好更直接。数学上：无限时间步的奖励之和可能不会收敛到有限的值，但是在有折扣因子的情况下一般是可以收敛的。

尽管这两种回报形式之间的差异非常明显，但Deep-RL实践中往往会混用。例如，我们经常设置算法来优化未折现收益，但在评估价值函数时使用折现因子。



## 6. 强化学习问题 The RL Problem

无论选择哪种方式衡量回报（$T$步累计奖赏或者 $\gamma$ 折扣奖励），无论选择哪种策略，强化学习的目标都是选择一种策略从而最大化回报的期望，即$E[R(\tau)]$。

讨论这个期望的表达式之前，我们先讨论下行动轨迹的概率分布。我们假设环境转换和策略都是随机的。这种情况下，$T$步行动轨迹的概率是$P(\tau|\pi) = \rho_0 (s_0) \prod_{t=0}^{T-1} P(s_{t+1} | s_t, a_t) \pi(a_t | s_t)$。

则回报的期望是$J(\pi) = \int_{\tau} P(\tau|\pi) R(\tau) = \mathop{E}_{\tau\sim \pi}{R(\tau)}$。

强化学习中的核心优化问题可以表示为：$\pi^* = \arg \max_{\pi} J(\pi)$。 $\pi^*$是**最优策略**，该策略可以最大会回报的期望。



## 7. 价值函数 Value Functions

了解状态或状态-动作对的值通常很有用。所谓价值，是指如果从某个状态或状态-动作对开始，然后再一直根据特定策略采取行动，最终的期望回报。几乎每种RL算法都在通过不同方式使用值函数。

这里介绍四种主要函数：

1. The **On-Policy Value Function** :

   > ![](https://spinningup.openai.com/en/latest/_images/math/e043709b46c9aa6811953dabd82461db6308fe19.svg)

2. The **On-Policy Action-Value Function**:

   > ![](https://spinningup.openai.com/en/latest/_images/math/85d41c8c383a96e1ed34fc66f14abd61b132dd28.svg)

3. The **Optimal Value Function**:

   > ![](https://spinningup.openai.com/en/latest/_images/math/01d48ea453ecb7b560ea7d42144ae24422fbd0eb.svg)

4. The **Optimal Action-Value Function**:

   > ![](https://spinningup.openai.com/en/latest/_images/math/bc92e8ce1cf0aaa212e144d5ed74e3b115453cb6.svg)



## 8. Bellman Backup

所有四个值函数都遵循称为Bellman方程的自洽方程。

贝尔曼期望方程：

![](https://spinningup.openai.com/en/latest/_images/math/7e4a2964e190104a669406ca5e1e320a5da8bae0.svg)

贝尔曼最优方程：

![](https://spinningup.openai.com/en/latest/_images/math/f8ab9b211bc9bb91cde189360051e3bd1f896afa.svg)

由不动点定理，通过不断backup迭代，均能收敛到策略价值和最优价值。