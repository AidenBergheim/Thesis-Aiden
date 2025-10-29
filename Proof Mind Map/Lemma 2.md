---

# Proof
---
---
$$\boldsymbol{\psi}(t) = \frac{\sum_{i=1}^{n} [\boldsymbol{x}_i - \boldsymbol{y}(t)]}{\| \sum_{i=1}^{n} [\boldsymbol{x}_i - \boldsymbol{y}(t)]\|}$$
# Proof
---
From **Background 2** we have

$$\boldsymbol{\psi}(t) = \frac{\boldsymbol{c} - \boldsymbol{y}(t)}{\|\boldsymbol{c} - \boldsymbol{y}(t)\|}$$

Using **Background 4** we get

$$\boldsymbol{\psi}(t) = \frac{\frac{1}{n} \sum_{i=1}^{n} \boldsymbol{x}_i - \boldsymbol{y}(t)}{\|\frac{1}{n} \sum_{i=1}^{n} \boldsymbol{x}_i - \boldsymbol{y}(t)\|} = \frac{\frac{1}{n} \sum_{i=1}^{n} \boldsymbol{x}_i - \frac{1}{n} \sum_{i=1}^{n}\boldsymbol{y}(t)}{\|\frac{1}{n} \sum_{i=1}^{n} \boldsymbol{x}_i - \frac{1}{n} \sum_{i=1}^{n}\boldsymbol{y}(t)\|}.$$
which simplifies to 

$$\boldsymbol{\psi}(t) = \frac{\sum_{i=1}^{n} [\boldsymbol{x}_i - \boldsymbol{y}(t)]}{\| \sum_{i=1}^{n} [\boldsymbol{x}_i - \boldsymbol{y}(t)]\|},$$
as required.