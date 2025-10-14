---

# Proof
---
---
Suppose **Assumption 1** holds, then under the target estimator and controller, the agent to targets centroid distance $d(t)$ is bounded by the interval $d^*(t) \leq d(t)\leq d_{max}(t)$ for  $t> T_{c,1}$. 
# Proof
---
We have using the **Dynamics** that

$$\dot{d}(t)=-u_\psi(t)=-\frac{\exp\!\big(|\tilde{d}(t)|^{\alpha_2}\big)}{\alpha_2 T_{c,2}}
\, \text{sig}^{1-\alpha_2}\!\big(\hat{d}(t) - d^*(t))+\dot{d}^*(t)$$
Since we have that $\|\boldsymbol{\tilde{x}}(t)\|=0$ for $t>T_{c,1}$, we get that $\hat{d}(t)=d(t)$ for $t>T_{c,1}$, hence we have that
$$\dot{d}(t)=\operatorname{sign}(d^*(t) - d(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |d(t) - d^*(t)|^{1-\alpha_2}+\dot{d}^*(t)$$



Hence, for the tracking error

$$\delta(t)=d(t)-d^*(t)$$

after taking the derivative wrt. time we have
$$\dot{\delta}(t)=\dot{d}(t)-\dot{d^*}(t)=\operatorname{sign}(-\delta(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |\delta(t)|^{1-\alpha_2}+\dot{d^*}(t)-\dot{d^*}(t)$$

$$\dot{\delta}(t)=\operatorname{sign}(-\delta(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |\delta(t)|^{1-\alpha_2}$$
Since we have shown that $d(T_{c,1})=d(0)>d^*(T_{c,1})$, and hence $\delta(T_{c,1})>0$ and we have that both $d(t)$ and $d^*(t)$ are continuous, it follows that since $\dot{\delta}(t)\geq 0$  for $d(t)\leq d^*(t)$, we must have that
$$d(t)\geq d^*(t)>0$$
for $t>T_{c,1}$.

Similarly, since $d(T_{c,1})=d(0)>d^*(T_{c,1})$ and $\dot{\delta}(t)\leq 0$  for $d(t)\geq d^*(t)$, we must have that 

$$d(t)\leq d_{max}(t)<\infty$$

for $t>T_{c,1}$ as well.
