$t \in \mathbb{R}$: time
$n \in \mathbb{Z^+}$: number of targets
$\boldsymbol{y}(t) \in \mathbb{R}^2$: position of agent
$\boldsymbol{u}(t) \in \mathbb{R}^2$: control input
$\boldsymbol{x}_i(t) \in \mathbb{R}^2$: position of target $i$
$\hat{\boldsymbol{x}}_i(t) \in \mathbb{R}^2$: estimated position of target $i$
$\tilde{\boldsymbol{x}}_i(t) \in \mathbb{R}^2$: error in estimation of target $i$ position $\hat{\boldsymbol{x}}_i(t) - \boldsymbol{x}_i(t)$
$\boldsymbol{c}(t) \in \mathbb{R}^2$: position of the centroid of the targets
$\hat{\boldsymbol{c}}(t) \in \mathbb{R}^2$: estimated position of the centroid of the targets
$\tilde{\boldsymbol{c}}(t) \in \mathbb{R}^2$: error in estimated position of the centroid of the targets $\hat{\boldsymbol{c}}(t) - \boldsymbol{c}(t)$
$d(t) \in \mathbb{R}$: distance between the agent and centroid of the targets $\boldsymbol{c}(t)$
$\hat{d}(t) \in \mathbb{R}$: estimated distance between the agent and the centroid
$\tilde{d}(t) \in \mathbb{R}$: the error in agent to target centroid estimation $\hat{d}(t) - d^*(t)$
$d^*(t) \in \mathbb{R}$: the desired circumnavigation distance from the targets centroid.
$d_s \in \mathbb{R}$: the minimum safe distance between the agent and the targets centroid
$\boldsymbol{\varphi}_i(t) \in \mathbb{R}^2$: unit vector from the agent to target $i$
$\boldsymbol{\bar{\varphi}}_i(t) \in \mathbb{R}^2$: unit vector perpendicular to $\boldsymbol{\varphi}_i(t)$obtained by a $\pi/2$ clockwise rotation of $\boldsymbol{\varphi}_i(t)$
$\boldsymbol{r}(t) \in \mathbb{R}^2$: the position of the agent relative to the targets centroid
$\hat{\boldsymbol{r}}(t) \in \mathbb{R}^2$: the estimated position of the agent relative to the targets centroid
$\tilde{\boldsymbol{r}}(t) \in \mathbb{R}^2$: the error in estimated position of the agent relative to the targets centroid $\hat{\boldsymbol{r}}(t) - \boldsymbol{r}(t)$
$\boldsymbol{\psi}(t) \in \mathbb{R}^2$: unit vector from the agent to the centroid of the targets $\boldsymbol{c}(t)$
$\boldsymbol{\bar{\psi}}(t) \in \mathbb{R}^2$: unit vector perpendicular to $\boldsymbol{\psi}(t)$ obtained by a $\pi/2$ clockwise rotation of $\boldsymbol{\psi}(t)$