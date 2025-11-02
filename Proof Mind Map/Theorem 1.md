
---
Suppose the **Assumptions** hold. Then under the target estimator and control algorithm, the norm of the target estimation error $\|\tilde{x}_i(t)\|$ and the norm of the centroid estimation error $\|\tilde{c}(t)\|$ converges to zero within strongly predefined time $T_{c,1}$.

# Proof
---
Let $m \geq 1$ and $p \in (0, 1/m]$ be auxiliary variables satisfying $mp = \alpha_1$. Consider the Lyapunov function $V_1 = \|\tilde{x}_i(t)\|^m$, whose time derivative along the system trajectories is obtained as, 

$$\begin{align} \dot{V}_1 & = m \|\tilde{x}_i(t)\|^{m-2} \tilde{x}_i^\top(t) \left( -\frac{\exp(\|\tilde{x}_i(t)\|^{\alpha_1})}{\alpha_1 T_{c,1}} \psi_i^{\alpha_1}(\tilde{x}_i(t)) \right) \nonumber \\ & = - \frac{m}{mpT_{c,1}} \|\tilde{x}_i(t)\|^{m(1-p)} \exp\left(\|\tilde{x}_i(t)\|^{mp}\right) \nonumber \\ & = - \frac{1}{pT_{c,1}} V_1^{1-p} \exp(V_1^p), \end{align}$$

which takes on the exact form of **Lemma 3**. Therefore, according to Lemma 3, the set $M_1 = \{\tilde{x} = 0_2\}$ is GSPTA with the strongly predefined time $T_{c,1}$. Hence, due to the relationship between $\boldsymbol{c}(t)$ and $\boldsymbol{x}_i(t)$ from **Background 4**, we must also have that the set $M_2 = \{\tilde{c} = 0_2\}$ is GSPTA with the strongly predefined time $T_{c,1}$.