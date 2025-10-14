
$$\dot{d}(t)=\frac{d}{dt}\sqrt{(\boldsymbol{p}(t)-\boldsymbol{y}(t))^\top(\boldsymbol{p}(t)-\boldsymbol{y}(t))}$$

Applying the chain rule we get

$$\dot{d}(t)=\frac{(\boldsymbol{p}(t)-\boldsymbol{y}(t))^\top(\boldsymbol{\dot{p}}(t)-\boldsymbol{\dot{y}}(t))}{d(t)}$$
Applying **Background 1** we have
$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-\boldsymbol{\dot{y}}(t))$$
$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-\boldsymbol{u}(t))$$
For $t\leq t_s$

$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-k_{\omega}\,\bar{\boldsymbol{\psi}}(t))$$

Since $\psi^\top\bar{\psi}\equiv0$, we get

$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)\boldsymbol{\dot{p}}(t)$$
**CANNOT SHOW THIS IS ZERO (BECAUSE IT WOULD NOT BE)**

For $t > t_s$

$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-u_\psi(t)\boldsymbol{\psi}(t)
- k_{\omega}\,\bar{\boldsymbol{\psi}}(t))$$

Since $\psi^\top\bar{\psi}\equiv0$, we get

$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-u_\psi(t)\boldsymbol{\psi}(t))$$

Since $\psi^\top\psi\equiv1$
$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)\boldsymbol{\dot{p}}(t)-u_\psi(t)$$




---
Suppose **Assumption 1** holds, then under the target estimator and controller, the agent to targets centroid distance $d(t)$ is bounded by the interval $d^*(t) \leq d(t)\leq d_{max}(t)$ for  $t> t_s$. 
# Proof
---
We have using the **Dynamics** that

$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)\boldsymbol{\dot{p}}(t)-u_\psi(t)=\boldsymbol{\psi}^\top(t)\boldsymbol{\dot{p}}(t)-\frac{\exp\!\big(|\tilde{d}(t)|^{\alpha_2}\big)}{\alpha_2 T_{c,2}}
\, \text{sig}^{1-\alpha_2}\!\big(\hat{d}(t) - d^*)$$
Since we have that $\|\boldsymbol{\tilde{x}}(t)\|=0$ for $t>t_s$, we get that $\hat{d}(t)=d(t)$ for $t>t_s$, hence we have that
$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)\boldsymbol{\dot{p}}(t)+\operatorname{sign}(d^* - d(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |d(t) - d^*|^{1-\alpha_2}$$
Since we have shown that $d(t_s)=d(0)>d^*(t)$, and we have that $d(t)$ is continuous, it follows that since $\dot{d}(t)\geq 0$  for $d(t)\leq d^*(t)$, we must have that
$$d(t)\geq d^*(t)$$
for $t>t_s$.



**Counter Example**

$$\delta(t)=d(t)-d^*(t)$$
$$\dot{\delta}(t)=\dot{d}(t)-\dot{d^*}(t)=\operatorname{sign}(-\delta(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |\delta(t)|^{1-\alpha_2}-\dot{d^*}(t)$$
When Tracking error is zero, that is $\delta(t)=d(t)-d^*(t)=0$, we have $\dot{d}(t)=0$ and hence
$$\dot{\delta}(t)=0-\dot{d^*}(t)=-\dot{d^*}(t)$$
If $\dot{d^*}(t)>0$, since $\dot{d}(t)=0$, that means that momentarily we may have 

$$d^*(t)>d(t)$$
i.e. the desired distance is larger than our actual so we are in the shape are circumnavigation