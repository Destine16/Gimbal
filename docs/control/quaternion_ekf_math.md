# Quaternion EKF 数学说明

## 总览

姿态估计器采用基于四元数的 EKF，底层依托 `kalman_filter` 框架。

最关键的数学部分有三块：

- 用陀螺仪推进四元数
- 用重力方向构造观测雅可比
- 用 Kalman 增益修正 6 维状态

## 状态向量

```text
x = [q0, q1, q2, q3, bx, by]^T
```

前四维是四元数，后两维是陀螺仪 x/y 轴零偏。

## 过程模型

连续时间四元数方程可以写成：

```math
\dot q = \frac{1}{2}\Omega(\omega^{corr}) q
```

其中修正后的角速度是：

```math
\omega^{corr} =
\begin{bmatrix}
\omega_x - b_x \\
\omega_y - b_y \\
\omega_z
\end{bmatrix}
```

矩阵形式中的 `\Omega(\omega)` 为：

```math
\Omega(\omega)=
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
```

做一阶离散化后：

```math
q_{k+1} \approx \left(I + \frac{1}{2}\Omega(\omega_k^{corr})dt\right) q_k
```

## 状态转移矩阵

完整 EKF 的状态转移矩阵是 `6x6`。

按分块写为：

```math
F =
\begin{bmatrix}
F_{qq} & F_{qb} \\
0      & I
\end{bmatrix}
```

含义如下：

- `Fqq`：四元数传播块
- `Fqb`：bias 误差对四元数误差传播的耦合块
- 左下角是 0：当前模型里 bias 传播不直接依赖四元数
- 右下角是单位阵：bias 近似按常值 / 慢变随机游走传播

## 一轮更新里出现的两份 F

同一轮更新里会出现两个时刻的 `F`：

1. `F_state`
   - 用于状态预测
   - 此时右上角 `4x2` 还是 0
2. `F_cov`
   - 用于协方差传播
   - 在得到先验四元数后，再补上右上角 `4x2`

这是当前实现在线性化工作点上的工程选择。

## 观测模型

加速度先做归一化，再作为重力方向观测使用。

预测量测为：

```math
h(x)=
\begin{bmatrix}
2(q_1q_3-q_0q_2) \\
2(q_0q_1+q_2q_3) \\
q_0^2-q_1^2-q_2^2+q_3^2
\end{bmatrix}
```

它表示当前姿态下，body 系中预测出来的重力方向。

## 观测雅可比

`H` 不是常量矩阵，而是在当前先验状态附近对 `h(x)` 求偏导得到的雅可比。

在本工程里：

- `H` 是 `3x6`
- 前四列对应对四元数的偏导
- 后两列是 0，因为 `h(x)` 不直接依赖 `bx`、`by`

## Kalman 增益

因为状态是 `6x1`，量测是 `3x1`，所以增益矩阵 `K` 是 `6x3``：

```math
K = P^- H^T (H P^- H^T + R)^{-1}
```

含义是：

- 每一行对应一个状态量
- 每一列对应一个残差分量

所以 `K` 的 18 个元素描述的是：3 维量测残差如何修正 6 维状态。

## 代码位置

主要对应文件：

- 过程模型和 EKF 主流程：
  - `Modules/algorithm/QuaternionEKF.c`
- 固定维度 KF 框架：
  - `Modules/algorithm/kalman_filter.c`

工程中的 `kalman_filter` 已裁剪为只保留 `QuaternionEKF` 需要的部分。
