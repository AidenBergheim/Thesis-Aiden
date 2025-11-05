
Suppose **Assumption 1** holds, then under the target estimator and controller, the agent to targets centroid distance $d(t)$ is $d(t)\equiv d(0)$ for  $t\leq t_s$. 
# Proof
---
Differentiating $r(t)$ wrt. time, using **Lemma 1**, we have
$$\boldsymbol{\dot{r}}(t)=\boldsymbol{\dot{c}}(t)-\boldsymbol{\dot{y}}(t)=-\boldsymbol{\dot{y}}(t)=-k_{\omega}\,\bar{\boldsymbol{\psi}}(t)$$
Differentiating again wrt. time, we get

$$\boldsymbol{\ddot{r}}(t)=-k_{\omega}\,\dot{\bar{\boldsymbol{\psi}}}(t)=-k_{\omega}\frac{d}{dt}\begin{bmatrix} \cos(\beta(t)) \\ \sin(\beta(t)) \end{bmatrix}=-k_{\omega}\frac{d\beta(t)}{dt}\begin{bmatrix} -\sin(\beta(t)) \\ \cos(\beta(t)) \end{bmatrix}$$
$$\boldsymbol{\ddot{r}}(t)=-k_{\omega}\dot{\beta}(t)\boldsymbol{\psi}(t)$$
Now, let $h(t)$ be an auxiliary function defined as
$$h(t) = \boldsymbol{r}^{\perp}(t) \dot{\boldsymbol{r}}(t)$$differentiating $h(t)$ wrt. time we have 
$$\begin{align*} \dot{h}(t) &= \frac{d}{dt}\left(\boldsymbol{r}^{\perp}(t)\right)\dot{\boldsymbol{r}}(t) + \boldsymbol{r}^{\perp}(t)\ddot{\boldsymbol{r}}(t) \\ &= \left(\dot{\boldsymbol{r}}\right)^{\perp}(t)\dot{\boldsymbol{r}}(t) + \boldsymbol{r}^{\perp}(t)\ddot{\boldsymbol{r}}(t) \\ &= \boldsymbol{r}^{\perp}(t)\ddot{\boldsymbol{r}}(t) \\ &= (d(t)\boldsymbol{\psi}(t))^{\perp}(-k_{\omega}\dot{\beta}(t)\boldsymbol{\psi}(t))\end{align*}$$
Since $\boldsymbol{\psi}^{\perp}\boldsymbol{\psi}=0$, we have
$$\dot{h}(t)=0$$
Hence, we have that $h(t)\equiv h(0)$.

We also have that
$$h(t) = (d(t)\boldsymbol{\psi}(t))^{\perp}(-k_{\omega}\,\bar{\boldsymbol{\psi}}(t))$$
Since $\boldsymbol{\psi}^\perp(t)=-\bar{\boldsymbol{\psi}}(t)$, we get

$$h(t) = d(t)k_\omega\bar{\boldsymbol{\psi}}^\top(t)\bar{\boldsymbol{\psi}}(t)=d(t)k_\omega$$
Since $k_\omega>0$ and from **Assumption 1** that $d(0)>d_s>0$, we get

$$h(0)>d(0)k_\omega>0$$
So, since $h(t)\equiv h(0)$, we have

$$d(t)=\frac{h(t)}{k_\omega}\equiv\frac{h(0)}{k_\omega}.$$
where $h(0)$ and  are both constants for all $t\leq t_s$. Hence we have that

$$d(t)\equiv d(0),$$
for all $t\leq t_s$, where $d(0)$ is the initial distance between the agent and targets centroid.