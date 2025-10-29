
---
The following is bounded
$$0<C_{1,i}<\frac{d\gamma_{\nu_i}(t)}{dt}\leq \frac{k_{\omega}}{\|\boldsymbol{x}_i(t) - \boldsymbol{y}(t)\|}=C_{2,i}<\infty$$
for $t\leq T_{c,1}$
# Proof
---
To start, we will show that the agents tangential velocity relative to each target is bounded. We have from the control law, that for all $t\leq T_{c,1}$, the agents speed takes the constant value $k_{\omega}$. 

Now let us define $v_{\perp,i} \in \mathbb{R}$ and $v_{\parallel,i} \in \mathbb{R}$ as the components of the agents velocity perpendicular and parallel to $\boldsymbol{\varphi}_i(t)$ respectively. Now, by the Pythagorean Theorem we have that $k_{\omega}^2=v_{\perp,i}^2 + v_{\parallel,i}^2$, and hence we must have that 
$$v_{\perp,i}\leq k_{\omega}.$$
Next, we employ proof by contradiction to prove that $v_{\perp,i}$ is never zero for $t\in[0, T_{c,1})$. For $v_{\perp,i}=0$ to be true, there must be at least a time $t=\mathcal{T} \in (0, T_{c,1}]$, for which the agents velocity is directed towards the target $\boldsymbol{x}_i$, or the agent and target occupy the same position. 

Due to the agent's constant circular orbit around the targets for all $t\leq T_{c,1}$, this can only be true if at least $\max\|\boldsymbol{x}_i - \boldsymbol{c}\|\geq d(0)$. However, this leads to a contradiction since from **Assumption 1** we have $d(0) > \max{\|\boldsymbol{x}_i - \boldsymbol{c}\|}+d_s>\max\|\boldsymbol{x}_i - \boldsymbol{c}\|$. Hence, we must have that
$$0<v_{\perp,i}\leq k_{\omega}.$$
Now, using the triangle inequality, we have that
$$\|\boldsymbol{x}_i - \boldsymbol{y}(t)\|+\|\boldsymbol{x}_i - \boldsymbol{c}\|\geq\|\boldsymbol{y}(t) - \boldsymbol{c}\|.$$

From Lemma 4 since, $\|\boldsymbol{y}(t) - \boldsymbol{c}\|=d(t)\equiv d(0)>d^*(0)$ for all $t\leq T_{c,1}$, we must have that

$$\|\boldsymbol{x}_i - \boldsymbol{y}(t)\|\geq\|\boldsymbol{y}(t) - \boldsymbol{c}\| - \|\boldsymbol{x}_i - \boldsymbol{c}\|>d^*(0)-\|\boldsymbol{x}_i - \boldsymbol{c}\|.$$
Since, from **Definition 1** we must have that  $$d^*(0)>\max\|\boldsymbol{x}_i - \boldsymbol{c}\|+d_s \geq\|\boldsymbol{x}_i - \boldsymbol{c}\|+d_s,$$
we get that
$$\|\boldsymbol{x}_i - \boldsymbol{y}(t)\|> d^*(0)-\|\boldsymbol{x}_i - \boldsymbol{c}\| >d_s > 0,$$
for all $t\leq T_{c,1}$. Hence, since the agent to target distance $\|\boldsymbol{x}_i - \boldsymbol{y}(t)\|$ is always positive, we can relate the bounds of the agents velocity, to bounds of the angular velocity $\frac{d\gamma_{\nu_i}(t)}{dt}$ as
$$\frac{d\gamma_{\nu_i}(t)}{dt}=\frac{v_{\perp,i}}{\|\boldsymbol{x}_i - \boldsymbol{y}(t)\|}=C_{1,i}>0.$$
$$\frac{d\gamma_{\nu_i}(t)}{dt}=\frac{v_{\perp,i}}{\|\boldsymbol{x}_i - \boldsymbol{y}(t)\|}\leq \frac{k_{\omega}}{\|\boldsymbol{x}_i - \boldsymbol{y}(t)\|}=C_{2,i}<\infty,$$
