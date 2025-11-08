---

# Proof
---
---
Suppose **Assumption 1** and **Assumption 2** holds, then under the target estimator and controller, the agent to targets centroid distance $d(t)$ is bounded by the interval $$0<d^*(t) \leq d(t)\leq d_{max}<\infty$$ for  $t> T_{c,1}$. 
# Proof
---
Assuming $d(t)>0$ for $t>T_{c,1}$, using the **Dynamics** we have that for $t>T_{c,1}$

$$\dot{d}(t)=-u_\psi(t)=-\frac{\exp\!\big(|\tilde{d}(t)|^{\alpha_2}\big)}{\alpha_2 T_{c,2}}
\, \text{sig}^{1-\alpha_2}\!\big(\hat{d}(t) - d^*(t))+\dot{d}^*(t).$$
Now, assuming we have that $\|\boldsymbol{\tilde{x}}(t)\|=0$ for $t>T_{c,1}$, we get that $\hat{d}(t)=d(t)$ for $t>T_{c,1}$, hence we have that
$$\dot{d}(t)=\operatorname{sign}(d^*(t) - d(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |d(t) - d^*(t)|^{1-\alpha_2}+\dot{d}^*(t)$$
Hence, for the tracking error

$$\delta(t)=d(t)-d^*(t)$$

after taking the derivative wrt. time we have
$$\dot{\delta}(t)=\dot{d}(t)-\dot{d^*}(t)=\operatorname{sign}(-\delta(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |\delta(t)|^{1-\alpha_2}+\dot{d^*}(t)-\dot{d^*}(t)$$
$$\dot{\delta}(t)=\operatorname{sign}(-\delta(t)) \frac{\exp(|\tilde{d}(t)|^{\alpha_2})}{\alpha_2 T_{c,2}} |\delta(t)|^{1-\alpha_2}$$

From **Lemma 4** and **Assumption 2** we have that $d(T_{c,1})=d(0)>d^*(T_{c,1})$, and hence $\delta(T_{c,1})>0$. We also have that both $d(t)$ and $d^*(t)$ are continuous, so it follows that in the case that the agent gets closer to the centroid than the desired distance, implying $d(t)\leq d^*(t)$, we have that $\dot{\delta}(t)\geq 0$  and hence we must have that
$$d(t)\geq d^*(t)>0$$
for $t>T_{c,1}$.

Similarly, since $d(T_{c,1})=d(0)>d^*(T_{c,1})$ and hence $\delta(T_{c,1})>0$ and we have that  $\dot{\delta}(t)\leq 0$  for $\delta(t)\geq 0$, we must have that for some finite $d_{max} \in \mathbb{R}$ that

$$d(t)\leq d_{max}<\infty$$

for $t>T_{c,1}$ as well.
