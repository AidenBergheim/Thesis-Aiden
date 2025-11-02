---

# Proof
---
---

# Problem 1 (Localization)
---
Design a target estimator $\dot{\hat{\boldsymbol{x}}}(t)=\dot{\hat{\boldsymbol{x}}}(\boldsymbol{y}(t), \boldsymbol{\varphi}_i(t))$ such that the agent localizes each stationary target $i$, using only the agents own location $\boldsymbol{y}(t)$ and the local bearing measurements to each target $\boldsymbol{\varphi}_i(t)$. Specifically, the estimation error $\tilde{\boldsymbol{x}}_i(t) =\hat{\boldsymbol{x}}_i(t) - \boldsymbol{x}_i(t)$ of each target should converge to the origin with the tunable strong predefined time $T_{c,1}>0$:
$$\forall i, \lim_{t \to T_{c,1}} \|\tilde{\boldsymbol{x}}_i(t)\| = \lim_{t \to T_{c,1}} \|\hat{\boldsymbol{x}}_i(t) - \boldsymbol{x}_i\| = 0.$$
# Problem 2 (Circumnavigation)
---
Design a circumnavigation controller $\boldsymbol{u}(t)=\boldsymbol{u}(\boldsymbol{y}(t), \boldsymbol{\varphi}_i(t), \tilde{\boldsymbol{c}}(t), d^*(t), k_\omega)$ such that after some tunable strong predefined time $T_{c,2} > T_{c,1} > 0$, the agent position converges to the desired shape of circumnavigation $d^*(t)$, that is

$$ \lim_{t \to T_{c,2}} \|\boldsymbol{c} - \boldsymbol{y}(t)\| = d^*(t),$$

while ensuring sustained motion with predefined speed $k_\omega$ for all $t>T_{c,1}$.