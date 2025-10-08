$t \in \mathbb{R}$: time
$n \in \mathbb{Z^+}$: number of targets
$\boldsymbol{y}(t) \in \mathbb{R}^2$: position of agent
$\boldsymbol{u}(t) \in \mathbb{R}^2$: control input
$\boldsymbol{x}_i(t) \in \mathbb{R}^2$: position of target $i$
$\hat{\boldsymbol{x}}_i(t) \in \mathbb{R}^2$: estimated position of target $i$
$\tilde{\boldsymbol{x}}_i(t) \in \mathbb{R}^2$: error in estimation of target$i$position $\hat{\boldsymbol{x}}_i(t) - \boldsymbol{x}_i(t)$
$\boldsymbol{c}(t) \in \mathbb{R}^2$: position of the centroid of the targets
$\hat{\boldsymbol{c}}(t) \in \mathbb{R}^2$: estimated position of the centroid of the targets
$\tilde{\boldsymbol{c}}(t) \in \mathbb{R}^2$: error in estimated position of the centroid of the targets $\hat{\boldsymbol{c}}(t) - \boldsymbol{c}(t)$
$d_i(t) \in \mathbb{R}$: distance between the agent and target $i$
$\hat{d}_i(t) \in \mathbb{R}$: estimated distance between the agent and target $i$
$\tilde{d}_i(t) \in \mathbb{R}$: the error in agent to target estimation $\hat{d}_i(t) - d_i(t)$
$\boldsymbol{\varphi}_i(t) \in \mathbb{R}^2$: unit vector from the agent to target $i$
$\boldsymbol{\bar{\varphi}}_i(t) \in \mathbb{R}^2$: unit vector perpendicular to $\boldsymbol{\varphi}_i(t)$obtained by a $\pi/2$ clockwise rotation of $\boldsymbol{\varphi}_i(t)$
$r(t) \in \mathbb{R}$: the minimum distance between the agent and shape of circumnavigation
$\hat{r}(t) \in \mathbb{R}$: the estimated minimum distance between the agent and shape of circumnavigation
$\tilde{r}(t) \in \mathbb{R}$: the error in estimated minimum distance between agent and shape of circumnavigation$\hat{r}(t) - r(t)$
$r^*(t) \in \mathbb{R}$: the desired circumnavigation distance from the shape of circumnavigation.
$r_s \in \mathbb{R}$: the minimum safe distance between the agent and a given target.
$\boldsymbol{\psi}(t) \in \mathbb{R}^2$: unit vector from the agent to the centroid of the targets $\boldsymbol{c}(t)$
$\boldsymbol{\bar{\psi}}(t) \in \mathbb{R}^2$: unit vector perpendicular to $\boldsymbol{\psi}(t)$ obtained by a $\pi/2$ clockwise rotation of $\boldsymbol{\psi}(t)$