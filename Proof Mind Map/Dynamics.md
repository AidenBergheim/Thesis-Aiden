
$$\dot{r}(t)=\frac{d}{dt}\sqrt{(\boldsymbol{c}(t)-\boldsymbol{y}(t))^\top(\boldsymbol{c}(t)-\boldsymbol{y}(t))}$$

Applying the chain rule we get

$$\dot{r}(t)=\frac{(\boldsymbol{c}(t)-\boldsymbol{y}(t))^\top(\boldsymbol{\dot{c}}(t)-\boldsymbol{\dot{y}}(t))}{r(t)}$$
Applying **Background 2** we have
$$\dot{r}(t)=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{c}}(t)-\boldsymbol{\dot{y}}(t))=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{c}}(t)-\boldsymbol{u}(t))$$

Applying **Lemma 1** we have
$$\dot{r}(t)=-\boldsymbol{\psi}^\top(t)\boldsymbol{u}(t)$$

For $t\leq t_s$

$$\dot{r}(t)=-\boldsymbol{\psi}^\top(t)k_{\omega}\,\bar{\boldsymbol{\psi}}(t)$$

Since $\psi^\top\bar{\psi}\equiv0$, we get

$$\dot{r}(t)=0$$

For $t > t_s$

$$\dot{r}(t)=-\boldsymbol{\boldsymbol{\psi}}^\top(t)(u_\psi(t)\boldsymbol{\psi}(t)
+ k_{\omega}\,\bar{\boldsymbol{\psi}}(t))$$

Since $\psi^\top\bar{\psi}\equiv0$, we get

$$\dot{r}(t)=-\boldsymbol{\boldsymbol{\psi}}^\top(t)u_\psi(t)\boldsymbol{\psi}(t)$$

Since $\psi^\top\psi\equiv1$
$$\dot{r(t)}=-u_\psi(t)$$