
---
The time derivative of the angle $\beta(t)$ along the trajectories of the system is given by $\frac{d\beta(t)}{dt}=\frac{k_{\omega}}{r(t)}$
# Proof (“Unbiased Bearing-Only Localization and Circumnavigation of a Constant Velocity Target,” Lemma 4.3)
---
***Result 1***

Taking time derivative of $\boldsymbol{\psi}(t)$, using **Background 1**

$$\boldsymbol{\dot{\psi}}(t) = \frac{d}{dt}\frac{\boldsymbol{c}(t)-\boldsymbol{y}(t)}{d(t)}$$
Applying produce rule
$$\boldsymbol{\dot{\psi}}(t) = \frac{(\dot{\boldsymbol{c}}(t)-\dot{\boldsymbol{y}}(t))d(t)-(\boldsymbol{c}(t)-\boldsymbol{y}(t))\dot{d}(t)}{d^2(t)}$$

Using **Background 1** again and **Lemma 1** we get

$$\boldsymbol{\dot{\psi}}(t) = \frac{-\dot{\boldsymbol{y}}(t)-\boldsymbol{\psi}(t)\dot{d}(t)}{d(t)}$$

When $t\leq t_s$, using **Dynamics** we have

$$\boldsymbol{\dot{\psi}}(t) = \frac{-\dot{\boldsymbol{y}}(t)}{d(t)}=-\frac{k_{\omega}}{d(t)}\bar{\boldsymbol{\psi}}(t)$$
When $t>t_s$ we get

$$\boldsymbol{\dot{\psi}}(t) = \frac{-u_\psi(t)\boldsymbol{\psi}(t)
- k_{\omega}\,\bar{\boldsymbol{\psi}}(t)+\boldsymbol{\psi}(t)u_\psi(t)}{d(t)}= -\frac{k_{\omega}}{d(t)}\bar{\boldsymbol{\psi}}(t)$$
So the dynamics governing $\boldsymbol{\dot{\psi}}(t)$ are the same for all $t\geq0$

***Result 2***

Using **Background 2** we also have

$$\boldsymbol{\dot{\psi}}(t)=\frac{d}{dt}\begin{bmatrix} -\sin(\beta(t)) \\ \cos(\beta(t)) \end{bmatrix}=\frac{d\beta(t)}{dt}\begin{bmatrix} -\cos(\beta(t)) \\ -\sin(\beta(t)) \end{bmatrix}$$

Using **Background 2** again we have
$$\boldsymbol{\dot{\psi}}(t)=-\frac{d\beta(t)}{dt}\boldsymbol{\bar{\psi}}(t)$$
Combining ***Result 1*** and ***Result 2*** we have

$$\frac{d\beta(t)}{dt}=\frac{k_{\omega}}{d(t)}$$
