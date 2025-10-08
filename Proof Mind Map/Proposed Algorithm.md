# Target Estimator
---

$$\dot{\hat{x}}_i(t) = -\frac{1}{\alpha_1 T_{c,1}} \exp\!\big(\|\xi_i(t)\|^{\alpha_1}\big) \phi_i^{\alpha_1}\!\big(\xi_i(t)\big)$$

- Control Constant: $\alpha_1 \in (0,1]$
- Predefined Time: $T_{c,1} > 0$
- Directing Function': $\phi^\beta(z) := \|z\|^{-\beta} z$ if $z \neq \mathbf{0}_n$, and $\phi^\beta(z) := \mathbf{0}_n$ if $z = \mathbf{0}_n$, for some $\beta \geq 0$ and $z \in \mathbb{R}^n$

$$\xi_i(t) =\begin{cases}P_i^{-1}(t)\big(P_i(t)\hat{x}_i(t) - q_i(t)\big), & t > 0, \\[6pt]\mathbf{0}_2, & t = 0,\end{cases}$$

$$ \begin{align*} \dot{P}_i(t) &= -P_i(t) + \bar{\varphi}_i(t)\bar{\varphi}_i^\top(t), & P_i(0) &= \mathbf{0}_{2\times 2}, \\ \dot{q}_i(t) &= -q_i(t) + \bar{\varphi}_i(t)\bar{\varphi}_i^\top(t)y(t), & q_i(0) &= \mathbf{0}_2, \end{align*} $$
# Control Algorithm
---

$$u_\psi(t) = \frac{\exp\!\big(|\tilde{d}(t)|^{\alpha_2}\big)}{\alpha_2 T_{c,2}}
\, \text{sig}^{1-\alpha_2}\!\big(\tilde{d}(t)\big)$$
$$u(t) =
\begin{cases}
k_{\omega}\,\bar{\boldsymbol{\psi}}(t), & t \leq t_s, \\[6pt]
u_\psi(t)\boldsymbol{\psi}(t)
+ k_{\omega}\,\bar{\boldsymbol{\psi}}(t), &    t > t_s,
\end{cases}$$

Where $t_s$ is the first time for which $\sum_{i=1}^{n} \left\| \tilde{x}_i(t) \right\|$ is less than some threshold value