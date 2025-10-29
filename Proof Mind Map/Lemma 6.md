---

# Proof
---
---
$P_i(t)$ is non-singular for all $0<t\leq T_{C,1}$
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

Converting the lower bound of the integral $h_{\boldsymbol{\nu_i}}$ in terms of $\theta_i$ we have

$$h_{\nu_i}(t_{0,i}, t_{0,i} + T_i) \geq e^{-T_i} \int_{\theta_{0,i}}^{\theta_{0,i} + \Theta_i} \cos^2(\theta_i) \frac{1}{\frac{d\gamma_{\nu_i}}{dt}} \, d\theta,$$
**The rest follow similarly from UnifiedSui2025 Lemma 2**