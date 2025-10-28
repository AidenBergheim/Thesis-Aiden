
---
Suppose the **Assumptions** hold. Then under the target estimator and control algorithm, the norm of the target estimation error $\delta(t)$ converges to zero within strongly predefined time $T_{c,2}$

# Proof
---
We have that the signal $\delta(t)=d(t)-d^*(t)$ must be bounded for $t\leq T_{c,1}$ since from Lemma 4, $d(t)\equiv d(0)$ for  $t\leq T_{c,1}$ and it is assumed that the desired distance to target $d^*(t)$ is bounded. Hence, we can consider the dynamics of the tracking error $\delta(t)$ for the phase $t>T_{c,1}$ as

$$\dot{\delta}(t)=\operatorname{sign}(-\delta(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |\delta(t)|^{1-\alpha_2}$$
Hence, introducing auxiliary variables $n \geq 1$ and $q \in (0, 1/n]$ such that $nq=\alpha_2$. Then, consider the Lyapunov candidate $V_2 = \|\delta(t)\|^n$, whose time derivative along the system trajectories is obtained as

$$\begin{align} \dot{V}_2 & = n |\delta(t)|^{n-2} \delta(t) \left( -\frac{\exp(|\delta(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} \text{sig}^{1-\alpha_2}(\delta(t)) \right) \nonumber \\ & = -\frac{n}{\alpha_2 T_{c,2}} |\delta(t)|^{n-2} \exp(|\delta(t)|^{\alpha_2}) |\delta(t)|^{2-\alpha_2} \nonumber \\ & = -\frac{n}{nq T_{c,2}} \exp(|\delta(t)|^{nq}) |\delta(t)|^{n(1-q)} \nonumber \\ & = -\frac{1}{q T_{c,2}} \exp(V_2^q) V_2^{1-q}, \end{align}$$
which takes on the exact form of **Lemma 3**. Therefore, according to Lemma 3, the set $M_2 = \{\delta = 0_2\}$ is GSPTA with the strongly predefined time $T_{c,2}$.