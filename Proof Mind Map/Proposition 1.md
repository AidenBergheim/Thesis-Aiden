
---
If $P_i(t_1)$ is positive definite at some time $t_1>0$, then for all $t>t_1$, $P_i(t)$ remains positive definite.
# Proof
---
We have for $t\geq0$ the solution
$$P_i(t) = e^{-t} P_i(0) + \int_{0}^{t} e^{-(t-s)} \bar{\boldsymbol{\varphi}}_i(s) \bar{\boldsymbol{\varphi}}_i^{\top}(s) \, ds=\int_{0}^{t} e^{-(t-s)} \bar{\boldsymbol{\varphi}}_i(s) \bar{\boldsymbol{\varphi}}_i^{\top}(s) \, ds.$$
So for $t \geq t_1$ we have 
$$P_i(t) = \underbrace{e^{-(t-t_1)} P_i(t_1)}_{=:A} + \underbrace{\int_{t_1}^{t} e^{-(t-s)} \bar{\boldsymbol{\varphi}}_i(s) \bar{\boldsymbol{\varphi}}_i^{\top}(s) \, ds}_{=:B}. $$
Term $A$ is positive definite by assumption and term $B$ is at least positive semi-definite for all $\boldsymbol{\varphi}_i(t) \in \mathbb{R}^2$. Since $P_i(t)$ can be expressed as the sum of the positive definite $A$ and positive semi-definite matrix $B$, $P_i(t)$ must be positive definite for all subsequent $t \geq t_1$.