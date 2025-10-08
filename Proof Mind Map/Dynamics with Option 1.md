
$$\dot{r(t)}=\frac{d}{dt}\sqrt{(\boldsymbol{p}(t)-\boldsymbol{y}(t))^\top(\boldsymbol{p}(t)-\boldsymbol{y}(t))}$$

Applying the chain rule we get

$$\dot{r(t)}=\frac{(\boldsymbol{p}(t)-\boldsymbol{y}(t))^\top(\boldsymbol{\dot{p}}(t)-\boldsymbol{\dot{y}}(t))}{r(t)}$$
Applying **Background 1** we have
$$\dot{r(t)}=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-\boldsymbol{\dot{y}}(t))$$
$$\dot{r(t)}=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-\boldsymbol{u}(t))$$
For $t\leq t_s$

$$\dot{r(t)}=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-k_{\omega}\,\bar{\boldsymbol{\psi}}(t))$$

Since $\psi^\top\bar{\psi}\equiv0$, we get

$$\dot{r(t)}=\boldsymbol{\psi}^\top(t)\boldsymbol{\dot{p}}(t)$$
**CANNOT SHOW THIS IS ZERO (BECAUSE IT WOULD NOT BE)**

For $t > t_s$

$$\dot{r(t)}=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-u_\psi(t)\boldsymbol{\psi}(t)
- k_{\omega}\,\bar{\boldsymbol{\psi}}(t))$$

Since $\psi^\top\bar{\psi}\equiv0$, we get

$$\dot{r(t)}=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{p}}(t)-u_\psi(t)\boldsymbol{\psi}(t))$$

Since $\psi^\top\psi\equiv1$
$$\dot{r(t)}=\boldsymbol{\psi}^\top(t)\boldsymbol{\dot{p}}(t)-u_\psi(t)$$