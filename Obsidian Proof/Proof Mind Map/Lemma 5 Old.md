---

# Proof
---
---
Suppose **Assumption 1** holds, then under the target estimator and controller, the agent to targets centroid distance $d(t)$ is bounded by the interval $d^*(t) \leq d(t)\leq d_{max}(t)$ for  $t> t_s$. 
# Proof
---
We have using the **Dynamics** that

$$\dot{d}(t)=-u_\psi(t)=-\frac{\exp\!\big(|\tilde{d}(t)|^{\alpha_2}\big)}{\alpha_2 T_{c,2}}
\, \text{sig}^{1-\alpha_2}\!\big(\hat{d}(t) - d^*(t))$$
Since we have that $\|\boldsymbol{\tilde{x}}(t)\|=0$ for $t>t_s$, we get that $\hat{d}(t)=d(t)$ for $t>t_s$, hence we have that
$$\dot{d}(t)=\operatorname{sign}(d^*(t) - d(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |d(t) - d^*(t)|^{1-\alpha_2}$$
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