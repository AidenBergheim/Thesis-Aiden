To derive the dynamics of $d(t)$, we can differentiate it wrt. time as follows
$$\dot{d}(t)=\frac{d}{dt}\sqrt{(\boldsymbol{c}(t)-\boldsymbol{y}(t))^\top(\boldsymbol{c}(t)-\boldsymbol{y}(t))}.$$

Applying the chain rule and assuming $d(t)>0$, we get

$$\dot{d}(t)=\frac{(\boldsymbol{c}(t)-\boldsymbol{y}(t))^\top(\boldsymbol{\dot{c}}(t)-\boldsymbol{\dot{y}}(t))}{d(t)}.$$
Applying **Background 2** we have
$$\dot{d}(t)=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{c}}(t)-\boldsymbol{\dot{y}}(t))=\boldsymbol{\psi}^\top(t)(\boldsymbol{\dot{c}}(t)-\boldsymbol{u}(t)).$$

Applying **Lemma 1** we have
$$\dot{d}(t)=-\boldsymbol{\psi}^\top(t)\boldsymbol{u}(t).$$

For $t\leq T_{c,1}$

$$\dot{d}(t)=-\boldsymbol{\psi}^\top(t)k_{\omega}\,\bar{\boldsymbol{\psi}}(t).$$

Since $\psi^\top\bar{\psi}\equiv0$, we get

$$\dot{d}(t)=0.$$

For $t >  T_{c,1}$

$$\dot{d}(t)=-\boldsymbol{\boldsymbol{\psi}}^\top(t)(u_\psi(t)\boldsymbol{\psi}(t)
+ k_{\omega}\,\bar{\boldsymbol{\psi}}(t)).$$

Since $\psi^\top\bar{\psi}\equiv0$, we get

$$\dot{d}(t)=-\boldsymbol{\boldsymbol{\psi}}^\top(t)u_\psi(t)\boldsymbol{\psi}(t).$$

Since $\psi^\top\psi\equiv1$
$$\dot{d}(t)=-u_\psi(t).$$