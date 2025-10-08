# Option 1: Closest Point
---
- $\boldsymbol{p}(t) \in \mathbb{R}^2$: position of the **closest point** on the shape of circumnavigation to the agent
- $r(t) \in \mathbb{R}$: the minimum distance between the agent and shape of circumnavigation $\|\boldsymbol{p}(t) - \boldsymbol{y}(t)\|$
- $\boldsymbol{\psi}(t) \in \mathbb{R}^2$: unit vector from the agent to the closest point $p(t)$ of the ellipse of circumnavigation

**Functionality**: we are trying to maintain a **constant distance** to the **constantly changing** closest point on the shape of circumnavigation, that is, $r(t)=r^*$ 


# Option 2: Targets Centroid
---
-  $\boldsymbol{c}(t) \in \mathbb{R}^2$: position of the centroid of the targets
- $r(t) \in \mathbb{R}$: the distance between the agent and target centroid $\|\boldsymbol{c}(t) - \boldsymbol{y}(t)\|$
- $\boldsymbol{\psi}(t) \in \mathbb{R}^2$: unit vector from the agent to targets centroid $\boldsymbol{c}(t)$

**Functionality**: we are trying to maintain a **varying distance** to the **constant** centroid point of the targets, that is, $r(t)=r^*(y(t))$ 

**Benefit**: The point for which we are circumnavigating is stationary (assuming **Assumption 2**)