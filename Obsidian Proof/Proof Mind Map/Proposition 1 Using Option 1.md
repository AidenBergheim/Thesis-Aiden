
---
The time derivative of the angle $\beta(t)$ along the trajectories of the system is given by $\frac{d\beta}{dt} =$
# Proof (“Unbiased Bearing-Only Localization and Circumnavigation of a Constant Velocity Target,” Lemma 4.3)
---
Taking time derivative of $\boldsymbol{\psi}(t)$, using **Background 1**

$$\boldsymbol{\dot{\psi}}(t) = \frac{d}{dt}\frac{\boldsymbol{p}(t)-\boldsymbol{y}(t)}{r(t)}$$
Applying produce rule
$$\boldsymbol{\dot{\psi}}(t) = \frac{(\dot{\boldsymbol{p}}(t)-\dot{\boldsymbol{y}}(t))r(t)-(\boldsymbol{p}(t)-\boldsymbol{y}(t))\dot{r}(t)}{r^2(t)}$$

Using **Background 1** again and since $\dot{\boldsymbol{c}}(t)=0$

$$\boldsymbol{\dot{\psi}}(t) = \frac{\dot{\boldsymbol{p}}(t)-\dot{\boldsymbol{y}}(t)-\boldsymbol{\psi}\dot{r}(t)}{r(t)}$$

So, since $\dot{r}(t)=0$ (theoretically just means we are circling the centroid of the targest, when $t\leq t_s$ we have

$$\boldsymbol{\dot{\psi}}(t) = \frac{\dot{\boldsymbol{p}}(t)-\dot{\boldsymbol{y}}(t)-\boldsymbol{\psi}\boldsymbol{\psi}^\top\dot{\boldsymbol{p}}(t)}{r(t)}$$
Since 

