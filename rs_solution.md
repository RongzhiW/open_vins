## Filter based

简化下问题，待估计状态中不对$t_r, t_d, T_{CI}$中估计.

feature的状态采用ANCHOR_MSCKF_INVERSE_DEPTH方式, 即:
$$
\lambda=
\begin{pmatrix}
\alpha\\
\beta\\
\rho\\
\end{pmatrix}=
\begin{pmatrix}
\frac{p_{A_x}}{p_{A_z}}\\
\frac{p_{A_y}}{p_{A_z}}\\
\frac{1}{p_{A_z}}
\end{pmatrix}\\
^Ap=\frac{1}{\rho}
\begin{pmatrix}
\alpha\\
\beta\\
1
\end{pmatrix}=h_4(\lambda)
$$
其中$^Ap$表示anchor camera frame下feature的三维坐标.

### 状态向量

$$
x=
\begin{pmatrix}
x^T_E& x_{I_1}^T& ...& x_{I_m}^T& \lambda_1^T& ...&\lambda_n^T
\end{pmatrix}^T
$$

其中$x_E=(t_{GI}\ q_{IG}\ v_G\ b_a\ b_g)^T$, 表示current imu state; $x_{I_n}=(t_{GI_n}\ q_{I_nG}\ v_{G_n})$, 表示第n帧image第一行对应的imu state.

### 观测方程

feature的观测方程:
$$
\begin{align}
z&=
\begin{pmatrix}
u\\
v
\end{pmatrix}+n\\
&=h(\lambda, x_{I_A}, x_{I_n}(t_v))+n\\
&=h_1(^Cp)=h_1(h_2(^Gp,x_{I_n}(t_v)))+n\\
&=h_1(h_2(h_3(^Ap,x_{I_A}),x_{I_n}(t_v)))+n\\
&=h_1(h_2(h_3(h_4(\lambda),x_{I_A}),x_{I_n}(t_v)))+n
\end{align}
$$
其中$x_{I_A}$为anchor frame对应的imu state, $x_{I_n}(t_v)$为第n帧第$v$行曝光对应的imu state, 

令$x_{I_n}(t_v)=(t_{GI_{nv}}\ q_{IG_{nv}}\ v_{G_{nv}})^T$, $x_{I_A}=(t_{GI_A}\ q_{IG_A}\ v_{G_A})^T$, 关于h_1,h_2,h_3$的表达式如下:
$$
h_1(^Cp)=\frac{K\cdot{^Cp}}{^Cp(2)}\\
h_2(^Gp,x_{I_n}(t_v))=q_{CI}\cdot q_{IG_{nv}}(^Gp-t_{GI_{nv}})+t_{CI}\\
h_3(^Ap,x_{I_A})=q_{IG_A}^{-1}\cdot q_{IC}({^Aq}-t_{CI})+t_{GI_A}
$$

### EKF propogation

$x_E$进行imu积分更新，下一帧image在$t_0$来到，于是进行state augment.

### State augmentation

$$
x=(x\ x_{I_{t0}})\\
P_x=
\begin{pmatrix}
P_x & P_{xx_{I_{t_0}}}\\
P_{x_{I_{t_0}}x} & P_{x_{I_{t0}}}
\end{pmatrix}
$$

其中
$$
x_{x_{I_{t_0}}}=(I_{7x7}\ 0_{7x9})\cdot x_E=Q\cdot x_E\\
P_{x_{I_{t0}}}=QP_{x_E}Q^T\\
P_{xx_{I_{t_0}}}=P_{xx_E}Q^T\\
P_{x_{I_{t_0}}x}=QP_{x_Ex}
$$

### EKF更新

观测方程的error model:
$$
\tilde z=H_{\lambda}\tilde\lambda+H_{x_{I_A}}\tilde x_{I_A}+H_{x_{I_{nv}}}\tilde x_{I_{nv}}+n
$$
其中
$$
H_{\lambda}=\frac{\partial h_1}{\partial {^Cp}}\cdot \frac{\partial h_2}{\partial {^Gp}}\cdot \frac{\partial h_3}{\partial {^Ap}}\cdot \frac{\partial h_4}{\partial \lambda}\\
H_{x_{I_A}}=\frac{\partial h_1}{\partial {^Cp}}\cdot \frac{\partial h_2}{\partial {^Gp}}\cdot \frac{\partial h_3}{\partial x_{I_A}}\\
H_{x_{I_{nv}}}=\frac{\partial h_1}{\partial {^Cp}}\cdot \frac{\partial h_2}{\partial x_{I_{nv}}}
$$
关于$h_1,h_2,h_3,h_4$的偏导公式详细参见[openvins理论推导][https://...]confluence页面.

由于状态空间没有$x_{I_{nv}}$, 只有$x_{I_n}$, 可将$x_{I_{nv}}$在$x_{I_n}$处泰勒展开,  实际观测方程只用到了$t_{GI}, q_{IG}$:
$$
\tilde t_{I_{nv}}=\tilde t_{I_n}+\frac{vt_r}{N}\tilde v_{I_n}+\frac{(vt_r)^2}{2N^2}\tilde a_{I_n} ...\\
\tilde \theta_{I_{nv}}=\tilde \theta_{I_{n}}+\frac{vt_r}{N} {^I\tilde w_{I_n}}+...
$$
采用$l_p=1,l_\theta=0$近似, 令$H_{x_{I_{nv}}}=(H_{t_{I_{nv}}}\ H_{\theta_{I_{nv}}}\ 0)$:
$$
\begin{align}
\tilde z&=H_{\lambda}\tilde \lambda+H_{x_{I_A}}\tilde x_{I_A}+H_{t_{I_{nv}}}\tilde t_{I_n}+H_{t_{I_{nv}}}\frac{vt_r}{N}\tilde v_{I_n}+H_{\theta_{I_{nv}}}\tilde \theta_{I_n}+n\\
&=H_{\lambda}\tilde \lambda+H_{x_{I_A}}\tilde x_{I_A}+H_{x_{I_n}}\tilde x_{I_n}+n
\end{align}
$$
其中$H_{x_{I_n}}=(H_{t_{I_{nv}}}\ H_{\theta_{I_{nv}}}\ H_{t_{nv}}\frac{vt_r}{N})$.

