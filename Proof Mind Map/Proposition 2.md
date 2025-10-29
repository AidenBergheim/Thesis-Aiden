
---
The minimum eigenvalue of the solution $P_i(t)$ is given by

$$\lambda_{min}(P_i(t)) = \min_{\boldsymbol{\nu_i} \in S^1} \underbrace{\int_{0}^{t} e^{-(t-s)} \cos^2(\gamma_{\boldsymbol{\nu_i}}(s)) \, ds}_{=:h_{\boldsymbol{\nu_i}}(0,t)}$$
where $S^1 \subset \mathbb{R}^2$  is a set defined as $S:=\{\boldsymbol{\nu_i} \in \mathbb{R}^2:\|\boldsymbol{\nu_i}\|=1\}$ and $\gamma_{\boldsymbol{\nu_i}}(s)$ denotes the angle measured in the counter clockwise direction from a unit vector $\boldsymbol{\nu_i} \in S^1$ to the vector $\bar{\boldsymbol{\varphi}}_i(t)$.

# Proof
---
Using the explicit solution from **Proposition 1**, the minimum eigenvalue of $P_i(t)$ can be expressed using the Rayleigh quotient as

$$\begin{align*} \lambda_{\min}(P(t)) &\stackrel{}{=} \min_{\boldsymbol{\nu} \in S^1} \boldsymbol{\nu}^\top P(t) \boldsymbol{\nu} \\ & \stackrel{\underline{(29)}}{=} \min_{\boldsymbol{\nu} \in S^1} \int_0^t e^{-(t-s)} \boldsymbol{\nu}^\top \bar{\boldsymbol{\varphi}}(s) \bar{\boldsymbol{\varphi}}^\top(s) \boldsymbol{\nu} \, ds \\ & \stackrel{}{=} \min_{\boldsymbol{\nu} \in S^1} \int_0^t e^{-(t-s)} \left( \boldsymbol{\nu}^\top \bar{\boldsymbol{\varphi}}(s) \right)^2 \, ds \\ & \stackrel{\|\boldsymbol{\nu}\|, \|\bar{\boldsymbol{\varphi}}\| = 1}{=} \min_{\boldsymbol{\nu} \in S^1} h_{\nu}(0, t). \end{align*}$$



