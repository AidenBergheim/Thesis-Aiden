---

# Proof
---
---
Suppose the Assumptions hold, then under the proposed control law, the solution $P_i(t)$ to the Kresisselmeier regressor extension scheme is non-singular for all $0<t\leq T_{C,1}$.
# Proof
---

From **Proposition 2**, it can be seen that a sufficient condition for ensuring $\lambda_{min}(P_i(t))>0$ is that the following inequality holds true for some constant $\mu_i>0$
$$ h_{\nu_i}(0, t) \ge \mu_i > 0, \quad \forall \boldsymbol{\nu_i} \in \mathcal{S}^1, t > 0. $$
To prove this, proof by contradiction will be employed. Assume, for contradiction, that there exists some constant $0<\mathcal{T_i}<T_{c,1}$, such that $P_i(t)$ is singular for all $t \in (0, \mathcal{T_i}]$. Then it follows from **Proposition 3** that the angular rate $\dot{\gamma}_{\nu_i}(t)$ is bounded by
$$0<C_{1,i}<\frac{d\gamma_{\nu_i}(t)}{dt}\leq C_{2,i}<\infty$$
It can also be seen that $t_{0,i}\geq 0$, $T_i>0$, $t_{0,i}+T_i<\mathcal{T_i}$, the angle $\gamma_{\nu_i}(t)$ is at least increased by a value of $C_{1,i}T_i$ over the interval $[t_{0,i}, t_{0,i}+T_i]$,
$$\gamma_{\nu_i}(t_{0,i} + T_i) \geq \gamma_{\nu_i}(t_{0,i}) +C_{1,i}T_{i}, \forall \boldsymbol{\nu_i} \in S^1.$$
Now, evaluating the integral $h_{\boldsymbol{\nu_i}}$on the sliding window $[t_{0,i}, t_{0,i}+T]$, we have
$$\begin{align*} h_{\nu_i}(t_{0,i}, t_{0,i} + T_i) & {=} \int_{t_{0,i}}^{t_{0,i}+T_i} e^{-(t_{0,i}+T_i-t)} \cos^2 (\gamma_{\nu_i}(t)) \, dt \\ & = e^{-t_{0,i}-T} \int_{t_{0,i}}^{t_{0,i}+T_i} e^t \cos^2 (\gamma_{\nu_i}(t)) \, dt \\ & \stackrel{e^t \geq e^{t_{0,i}}}{\geq} e^{-t_{0,i}-T_i} e^{t_{0,i}} \int_{t_{0,i}}^{t_{0,i}+T_i} \cos^2 (\gamma_{\nu_i}(t)) \, dt \\ & = e^{-T_i} \int_{t_{0,i}}^{t_{0,i}+T_i} \cos^2 (\gamma_{\nu_i}(t)) \, dt. \end{align*}$$
Consider a change of variable given by
$$\theta_i:=\gamma_{\nu_i}(t)$$
for $t\in [t_{0,i}, t_{0,i}+T]\subset [0,\mathcal{T_i}]$. Since $\gamma_{\nu_i}(t)$ is continuously differentiable, and $\frac{d\gamma_{\nu_i}(t)}{dt}>C_{1,i}>0$, we get that
$$\frac{d\theta_i}{dt} = \frac{d\gamma_{\nu_i}}{dt} \Longrightarrow d\theta_i = \left(\frac{d\gamma_{\nu_i}}{dt}\right) dt \Longrightarrow dt = \frac{d\theta_i}{\frac{d\gamma_{\nu_i}}{dt}}.$$

Converting the lower bound of the integral $h_{\boldsymbol{\nu_i}}$ in terms of $\theta_i$ instead of $t$, we have
$$h_{\nu_i}(t_{0,i}, t_{0,i} + T_i) \geq e^{-T_i} \int_{\theta_{0,i}}^{\theta_{0,i} + \Theta_i} \cos^2(\theta_i) \frac{1}{\frac{d\gamma_{\nu_i}}{dt}} \, d\theta_i,$$
where $\theta_0 :=\gamma_{\nu_i}(t_{0,i})$ and $\Theta_i:=\gamma_{\nu_i}(t_{0,i}+T)-\gamma_{\nu_i}(t_{0,i})$. Now, using the bounds of $\frac{d\gamma_{\nu_i}(t)}{dt}$, we have
$$0 < \frac{1}{C_{2,i}} \le \frac{d\gamma_{\nu_i}}{dt} \le \frac{1}{C_{1,i}} < \infty.$$
So, we get that

$$\begin{align*} h_{\nu_i}(t_{0,i}, t_{0,i} + T_i) &\stackrel{}{\ge} \frac{e^{-T_i}}{C_{2,i}} \int_{\theta_{0,i}}^{\theta_{0,i} + \Theta_i} \cos^2(\theta_i)  \, d\theta_i \\ &\stackrel{}{\ge} \frac{e^{-T_i}}{C_{2,i}} \int_{\theta_{0,i}}^{\theta_{0,i} + C_{1,i} T_i} \cos^2(\theta_i) \, d\theta_i.\end{align*}$$
Next, recall that for any length $L>0$, the minimum value of the integral $\int \cos^2(\theta) d\theta$ evaluates to
$$\min_{l \in \mathbb{R}} \int_{l}^{l+L} \cos^2(\theta) d\theta = \frac{1}{2}(L - |\sin(L)|).$$
Hence, the integral $h_{\nu_i}$ evaluated on the window $[t_{0,i}, t_{0,i} + T_i]$ is lower bounded by
$$h_{\nu}(t_{0,i}, t_{0,i} + T_i) \ge \underbrace{\frac{e^{-T_i}}{2C_{2,i}}(C_{1,i} T_i - |\sin(C_{1,i} T_i)|)}_{:=\eta(T_i)} > 0.$$
Hence, we get the inequality relating to the minimum eigenvalue of the matrix $P_i(t)$
$$\begin{align*} \lambda_{min}(P_i(t)) &\stackrel{}{\ge} \lambda_{min}(h_{\nu_i}(0, t)) \\ &= \lambda_{min}(h_{\nu_i}(0, T_i) + h_{\nu_i}(T_i, t)) \\ &\stackrel{}{\ge} \lambda_{min}(\eta(T_i)I + \eta(t-T_i)I) > 0, \quad \forall t \in [T_i, \mathcal{T}_i], \end{align*}$$
which contradicts the initial assumption that $P_i(t)$ is singular for all $t \in (0, \mathcal{T}]$. Hence, using proof by contradiction, there exists no constant $0<\mathcal{T_i}<T_{c,1}$ such that $P_i(t)$ is singular during $t \in (0, \mathcal{T}_i]$. Specifically we have that Eq. (10), holds uniformly in time for $t<T_{c,1}$ as $t_{0,i}$ and $T_i$ can be chosen arbitrarily and $\eta(T_i)$ is strictly greater than zero for any positive $T_i$. 

So, consider an arbitrary time $t_{a,i} < T_{c,1}$, and the window $T_{a,i}=t_{a,i}/2$, it follows that 
$$\lambda_{min}(P_i(T_{a,i})) \ge \lambda_{min}\left(2\eta\left(T_{a,i}\right)I \right) > \eta(T_{a,i})>0.$$
Since this holds for any $t_{a,i} < T_{c,1}$, we conclude that $P_i(t)$ must become non-singular immediately after $t$ becomes greater than 0. Now, from **Proposition 1**, since once $P_i(t)$ becomes non-singular, it must remain non-singular for $t\leq T_{c,1}$, we have that $P_i(t)$ remains non singular for the interval $0<t\leq T_{c,1}$.
