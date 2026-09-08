# Linearisation in Basalt

## 1. Introduction

Every estimation problem solved in the `basalt` back end is a nonlinear least squares problem posed over a manifold. None of them can be solved directly. The residuals are nonlinear functions of poses that live in $SE(3)$, of landmarks parameterised by a bearing and an inverse distance, and of inertial preintegration terms that depend on the biases through a first order correction. The only practical route to a solution is to replace the nonlinear problem, in a neighbourhood of the current estimate, by a linear one whose minimiser can be computed in closed form, then to repeat. That replacement is linearisation, and it is the subject of this document.

`doc/Marginalisation.md` §2 already establishes what linearisation produces, namely the normal equations $\mathbf{H}\xi = \mathbf{b}$ with $\mathbf{H} = \mathbf{J}^T\mathbf{W}\mathbf{J}$ and $\mathbf{b} = -\mathbf{J}^T\mathbf{W}\mathbf{r}$. `doc/VIO.md` §2.2.4, §2.2.5, §3.1.1 and §3.1.2 already establish what goes into $\mathbf{J}$ and $\mathbf{r}$, namely the reprojection residual and the inertial residual together with their analytic Jacobians. What neither document explains is the machinery in between, which is how those residuals and Jacobians are actually built, where they are stored, in what order the variables are laid out, how the landmarks are eliminated before the system is solved, and how the prior inherited from the previous marginalisation is folded back in. That machinery is a substantial body of code with its own abstractions, its own naming conventions and its own numerical reasoning, and this document is its reference.

The central design decision that governs all of it is that `basalt` does not, by default, form $\mathbf{H}$ at all when eliminating landmarks. It works instead with a square root of the information, that is with the Jacobian itself, and eliminates landmarks by an orthogonal transformation rather than by an algebraic inverse. This is the square root, or QR, formulation introduced by Demmel and colleagues, and it is selected by the default configuration value `LinearizationType::ABS_QR`. Understanding why that choice is made, what it costs and what it buys is the intellectual core of this document.

### 1.1 The Gauss-Newton Residual and the Linearised System

The objective in every case has the form of one half of a squared, weighted residual norm, taken over a state $\mathbf{s}$ that lives on a manifold.

$$E(\mathbf{s}) = \frac{1}{2}\mathbf{r}(\mathbf{s})^T\mathbf{W}\,\mathbf{r}(\mathbf{s})$$

A first order Taylor expansion about the current estimate, in terms of an increment $\xi$ applied through the manifold operator $\oplus$, gives the linearised residual.

$$\mathbf{r}(\mathbf{s} \oplus \xi) \approx \mathbf{r}(\mathbf{s}) + \mathbf{J}\xi, \qquad \mathbf{J} = (.\frac{\partial \mathbf{r}}{\partial \xi})|_{\mathbf{s}}$$

Substituting and minimising over $\xi$ yields the normal equations, derived in full in `doc/Marginalisation.md` §2.1 and not repeated here.

$$\mathbf{H}\xi = \mathbf{b}, \qquad \mathbf{H} = \mathbf{J}^T\mathbf{W}\mathbf{J}, \qquad \mathbf{b} = -\mathbf{J}^T\mathbf{W}\mathbf{r}$$

Two observations about this pair of equations set up everything that follows. The first is that $\mathbf{W}$ never needs to appear as a matrix. Since it is block diagonal and, for the visual factors, a scalar multiple of the identity, its square root can be absorbed into the residual and the Jacobian row by row, leaving an unweighted problem in the whitened quantities $\tilde{\mathbf{r}} = \mathbf{W}^{1/2}\mathbf{r}$ and $\tilde{\mathbf{J}} = \mathbf{W}^{1/2}\mathbf{J}$. `basalt` does exactly this, at `include/basalt/linearization/landmark_block_abs_dynamic.hpp:171`, and from that point onward the problem is a plain least squares problem $\min_\xi \|\tilde{\mathbf{J}}\xi + \tilde{\mathbf{r}}\|^2$. The tilde is dropped hereafter and every $\mathbf{J}$ and $\mathbf{r}$ in this document is understood to be whitened.

The second observation is that the step from $\mathbf{J}$ to $\mathbf{H}$ is not free. Forming $\mathbf{J}^T\mathbf{J}$ squares the condition number of the problem, which halves the number of significant digits available to the solver. Any algorithm that can operate on $\mathbf{J}$ directly, without ever forming $\mathbf{H}$, is numerically preferable. The QR based landmark elimination described in §3.1 is precisely such an algorithm, and §2.8 quantifies the argument.

### 1.2 The Three Problems That Employ Linearisation

Three distinct estimation problems in this codebase require a nonlinear least squares solution. They share the same mathematical shape but not the same implementation, and it is worth stating at the outset which of them actually use the machinery documented here.

#### 1.2.1 Visual-Inertial Odometry

The sliding window estimator minimises a sum of three residual families over the active window, comprising the reprojection residuals of every landmark observation, the preintegrated inertial residuals between consecutive states together with the bias random walk residuals, and the marginalisation prior inherited from the past. The full objective is assembled in `doc/VIO.md` §3.1.4 and reads, in the notation of that document, as follows.

$$E(\mathbf{s}) = \underbrace{\sum_{(i,j)} \frac{1}{2}\|\mathbf{r}^\text{vis}_{ij}\|^2_{\Sigma_\text{vis}}}_{\text{reprojection}} + \underbrace{\sum_k \frac{1}{2}\|\mathbf{r}^\text{imu}_k\|^2_{\Sigma_k}}_{\text{inertial}} + \underbrace{\sum_k \frac{1}{2}\|\mathbf{r}^\text{bias}_k\|^2}_{\text{bias random walk}} + \underbrace{E_\text{marg}(\mathbf{s})}_{\text{prior}}$$

The state comprises the six degree of freedom poses $\mathbf{T}_{wi}$ of older keyframes, the fifteen degree of freedom navigation states of recent frames holding pose, velocity and the two biases, and the three parameter landmarks. This problem is solved by `SqrtKeypointVioEstimator::optimize`, and it uses the linearisation framework documented here. `doc/VIO.md` §3.1.1 through §3.1.5 give the residuals and Jacobians in full.

#### 1.2.2 Marginalisation

Marginalisation solves no optimisation problem of its own. It linearises a sub problem, namely the residuals that touch the variables about to be removed, and then eliminates those variables algebraically to leave a prior over the survivors. The mathematics of the elimination, through the Schur complement, is given in `doc/Marginalisation.md` §3.2, and the derivation of the resulting synthetic residual is given in its Appendix A. What concerns the present document is the step before the elimination, which is the construction of the linear system that the elimination consumes. That step uses the same linearisation framework as the odometry problem, invoked with different arguments, and this shared use is the reason the two processes are documented together.

#### 1.2.3 Local Mapping

Local mapping is included here for contrast and to prevent a false inference. `class LocalMapper` extends `NfrMapper`, which extends `ScBundleAdjustmentBase<double>`, and it does not use the linearisation framework described in this document at all. Its objective, stated in `doc/Mapping.md` §3.1, combines visual reprojection residuals with the relative pose and roll-pitch factors recovered by nonlinear factor recovery.

$$E(\mathbf{s}) = E_\text{vision}(\mathbf{s}) + E_\text{rel}(\mathbf{s}) + E_\text{rp}(\mathbf{s})$$

It is solved by the classical Schur complement path through `ScBundleAdjustmentBase::linearizeHelper`, described in `doc/Mapping.md` §3.6.1 and §3.6.2, and `doc/LocalMapper.md` §7.7 confirms that the optimisation and outlier filtering routines are inherited unchanged. A search of `src/vi_estimator/local_mapper.cpp` for `LinearizationBase`, `performQR` or `LandmarkBlock` returns nothing. The square root formulation is therefore an odometry back end innovation only, and any future work to bring it to the mapper would be a genuine port rather than a configuration change.

### 1.3 Linearisation Overview

At a chosen timestamp the estimator holds a window of states and a database of landmarks with their observations. Linearisation turns that state of affairs into a matrix problem in five steps discussed below.

The variables are first placed in a definite order. An `AbsOrderMap` assigns to every frame in the window a column offset and a block width, six for a pose only keyframe and fifteen for a full navigation state, and records the total width of the system. Every Jacobian written thereafter is written at an offset dictated by this map, so the map is the single source of truth for the layout of the linear system.

The geometry shared between observations is then computed once. For every pair of a host frame and a target frame that share at least one landmark observation, the relative pose and its two Jacobians with respect to the host and target absolute poses are computed and cached. Since many landmarks are seen by the same pair of frames, this converts a per observation cost into a per frame pair cost.

Each landmark is then linearised independently into its own dense block, holding the pose Jacobian, the landmark Jacobian and the residual for all of that landmark's observations, stacked. Independence across landmarks is what allows this stage to be parallelised, and it is the structural fact that the whole design exploits.

The landmarks are then eliminated. In the default configuration each landmark block is left multiplied in place by an orthogonal matrix that annihilates its three landmark columns below the diagonal. The rows below the third are thereby made independent of that landmark entirely, and constitute the landmark's contribution to a reduced system in the poses alone. No inverse is computed and no information is lost.

Finally the reduced landmark rows, the inertial rows and the rows of the marginalisation prior are assembled into one system over the active variables, which is handed to a solver or, in the marginalisation case, to a second elimination.

The distinction between linearisation at $t = 0$ and at an arbitrary $t$ lies entirely in the last of these. At $t = 0$ there is no accumulated history, and the prior is a hand written diagonal matrix over a single state whose sole purpose is to fix the four unobservable gauge directions and to keep the biases from wandering. At any later $t$ the prior is a dense matrix produced by the previous marginalisation, coupling every surviving variable, and it must be re-centred at every use because the variables it constrains have moved since it was built. §3.2 develops both cases.

### 1.4 Document Structure

§2 assembles the mathematical preliminaries, seventeen in all, each given a definition, a derivation and a statement of its significance to the implementation. It opens with a dependency table so that a reader pursuing one question can take the shortest path to it. §3 develops the linearisation itself, first in its general absolute QR form in §3.1, which is the crux of the document, and then as applied to odometry in §3.2 and to marginalisation in §3.3. The integration of the previous marginalisation into the next optimisation is developed in §3.2.3. §4 turns to the software, describing the class hierarchy, the interface contract, the state machine, the input data, the three strategies, the threading model and a worked example, followed by the two call sites. §5 is the API appendix. §6 is the bibliography, to which every citation in the document is linked. §7 maps the relationship to the companion documents.

A reader who wants only the mechanism, and is willing to take the mathematics on trust, can read §1.3, then §3.1.5 and §3.1.6 for the storage layout and the elimination, then §3.2.3 for the prior, then §4.1.8 for the call sequence. A reader who wants the mathematics should read §2 in order first.

---

## 2. Mathematical Preliminaries

This section assembles every mathematical idea the linearisation machinery depends upon. It is written to be read in order, since each concept is used by the ones that follow, and it is deliberately self contained so that a reader can work through §3 and §4 without leaving the document. Four preliminaries are already treated elsewhere and are cited rather than repeated, namely quadratic forms and the information form of a Gaussian in `doc/Marginalisation.md` §A.1.1, the requirement that a Gauss-Newton solver be handed a sum of squares in §A.1.2, the existence of a square root of a positive semi-definite matrix in §A.1.3, and the Moore-Penrose pseudoinverse in §A.1.4.

The following table gives the dependency structure, so that a reader pursuing one question can take the shortest path to it.

| Concept | Depends on | Used by |
|---|---|---|
| §2.1 Least squares on a manifold | — | everything |
| §2.2 Weighted least squares and whitening | §2.1 | §2.3, §2.6 |
| §2.3 The Gauss-Newton method | §2.1, §2.2 | §2.4, §3.1 |
| §2.4 Damping and trust regions | §2.3 | §3.1.7, §3.2.4 |
| §2.5 Robust cost and reweighting | §2.2 | §3.1.4 |
| §2.6 Orthogonal matrices | §2.2 | §2.7 through §2.9, §2.11 |
| §2.7 QR decomposition | §2.6 | §2.11, §3.1.6, §3.3.3 |
| §2.8 Householder reflections | §2.7 | §3.1.6, §3.3.3 |
| §2.9 Givens rotations | §2.7 | §3.1.7 |
| §2.10 The Schur complement | §2.3 | §2.11, §3.3 |
| §2.11 Nullspace projection | §2.7, §2.10 | §3.1.6 |
| §2.12 Cholesky and LDLT | §2.10 | §3.2.4, §3.3.3 |
| §2.13 Conditioning | §2.7, §2.12 | the whole design rationale |
| §2.14 Column scaling | §2.13 | §3.1.8 |
| §2.15 Gauge freedom and observability | §2.10 | §2.16, §3.2.2 |
| §2.16 First-Estimate Jacobians | §2.15 | §3.1.3, §3.2.3 |
| §2.17 Marginalisation as conditioning | §2.10, §2.15 | §3.3 |

### 2.1 The Nonlinear Least Squares Problem on a Manifold

#### Definition

A nonlinear least squares problem seeks the state that minimises the sum of squared residuals, where a residual is a vector valued function measuring the disagreement between a prediction and a measurement. When the state is a vector in $\mathbb{R}^n$ the problem is stated as the minimisation of $\frac{1}{2}\|\mathbf{r}(\mathbf{s})\|^2$ over $\mathbf{s} \in \mathbb{R}^n$. In visual-inertial estimation the state is not a vector, because rotations and rigid body poses do not form a vector space, and the problem must be restated on a manifold.

#### Derivation

The difficulty is concrete. A rigid body pose is an element of the special Euclidean group $SE(3)$, comprising a rotation matrix and a translation. The set of rotation matrices is defined by the constraints $\mathbf{R}^T\mathbf{R} = \mathbf{I}$ and $\det\mathbf{R} = 1$, which are nonlinear equalities. The sum of two rotation matrices is not a rotation matrix, so the ordinary notion of adding an increment to a state fails immediately. Any parameterisation by three numbers, such as Euler angles, has singularities, and any parameterisation without singularities, such as a unit quaternion or a rotation matrix, is over parameterised and constrained.

The resolution used throughout modern estimation is to keep the state on the manifold and to represent only the increment in a vector space. A matrix Lie group $G$ has an associated Lie algebra $\mathfrak{g}$, which is a vector space of the same dimension as the group, being three for $SO(3)$ and six for $SE(3)$. The two are connected by the exponential map, which takes an element of the algebra to an element of the group, and by its inverse the logarithm. For $SO(3)$ the exponential map is given in closed form by the Rodrigues formula, where $\boldsymbol{\omega} \in \mathbb{R}^3$ is the rotation vector, $\theta = \|\boldsymbol{\omega}\|$ and $[\cdot]_\times$ is the skew symmetric matrix of a vector.

$$\exp([\boldsymbol{\omega}]_\times) = \mathbf{I} + \frac{\sin\theta}{\theta}[\boldsymbol{\omega}]_\times + \frac{1-\cos\theta}{\theta^2}[\boldsymbol{\omega}]_\times^2$$

This furnishes a local parameterisation. Near any group element $\bar{\mathbf{T}}$ every nearby element can be written as $\exp(\hat{\xi})\,\bar{\mathbf{T}}$ for a unique small $\xi \in \mathbb{R}^6$, where $\hat{\cdot}$ takes the vector to the algebra. The composition operator, conventionally written $\oplus$ and sometimes $\boxplus$, is therefore defined by the following, and the corresponding difference operator $\ominus$ recovers the increment by $\mathbf{T}_1 \ominus \mathbf{T}_2 = \log(\mathbf{T}_1\mathbf{T}_2^{-1})$.

$$\mathbf{T} \oplus \xi = \exp(\hat{\xi})\,\mathbf{T}$$

The choice to place the exponential on the left is a convention, and the alternative right convention $\mathbf{T}\exp(\hat{\xi})$ is equally valid but produces different Jacobians. `basalt` uses a decoupled convention in which the translation is updated additively and the rotation multiplicatively on the left, visible in `PoseState::incPose` in `thirdparty/basalt-headers/include/basalt/imu/imu_types.h:98-101`.

$$\mathbf{p}' = \mathbf{p} + \boldsymbol{\upsilon}, \qquad \mathbf{R}' = \exp([\boldsymbol{\omega}]_\times)\,\mathbf{R}$$

With this in hand the optimisation problem becomes well posed. At each iteration the current estimate $\bar{\mathbf{s}}$ is held fixed and the unknown is the increment $\xi$, which lives in an ordinary vector space of dimension equal to the number of degrees of freedom. The problem solved is therefore the minimisation over $\xi \in \mathbb{R}^n$ of the following, after which the estimate is replaced by $\bar{\mathbf{s}} \oplus \xi^\ast$ and the process repeats.

$$E(\xi) = \frac{1}{2}(\|\mathbf{r}(\bar{\mathbf{s}} \oplus \xi))\|^2$$

The Jacobian is likewise defined with respect to the increment rather than with respect to any parameterisation of the state itself, which is what makes it well defined and singularity free.

$$\mathbf{J} = (.\frac{\partial\,\mathbf{r}(\bar{\mathbf{s}} \oplus \xi)}{\partial \xi})|_{\xi = 0}$$

#### Significance

Three consequences run through the whole implementation.

The first is that every Jacobian in this codebase is a derivative with respect to a tangent increment, never with respect to a raw parameter. When the document writes $\partial\mathbf{r}/\partial\xi_h$ for the derivative with respect to a host pose, the six columns are the tangent directions of $SE(3)$ at that pose, ordered as three translation components followed by three rotation components.

The second is that the increment vector is what the linear system solves for, and it is the object laid out by the `AbsOrderMap` of §3.1.1. The state itself is never assembled into a vector anywhere in the code, because it could not be. Only increments are stacked.

The third is that the update is applied through a method rather than by addition. The methods are `PoseStateWithLin::applyInc` and `PoseVelBiasStateWithLin::applyInc`, and they are the only places where the manifold structure is exercised.

For a systematic treatment of Lie group state estimation the reader is referred to Sola and colleagues [[22]](#bib-22) and to Barfoot [[23]](#bib-23), and for the specific case of inertial preintegration on the manifold to Forster and colleagues [[6]](#bib-6).

### 2.2 Weighted Least Squares, the Mahalanobis Norm and Whitening

#### Definition

A weighted least squares problem minimises $\frac{1}{2}\mathbf{r}^T\mathbf{W}\mathbf{r}$ for a symmetric positive definite weight matrix $\mathbf{W}$ . Whitening is the transformation of such a problem into an unweighted one by absorbing a square root of $\mathbf{W}$ into the residual.

#### Derivation

The weight matrix is not a tuning device. It arises from maximum likelihood estimation under a Gaussian noise model, and its correct value is dictated by the sensor. Suppose a measurement $\mathbf{z}$ is related to the state by $\mathbf{z} = \mathbf{h}(\mathbf{s}) + \boldsymbol{\epsilon}$ with $\boldsymbol{\epsilon} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma})$. The likelihood of the measurement given the state is the Gaussian density, and its negative logarithm is the following, discarding the normalising constant.

$$-\log p(\mathbf{z}\mid\mathbf{s}) = \frac{1}{2}((\mathbf{z}-\mathbf{h}(\mathbf{s})))^T\boldsymbol{\Sigma}^{-1}((\mathbf{z}-\mathbf{h}(\mathbf{s}))) + \text{const}$$

Writing $\mathbf{r} = \mathbf{h}(\mathbf{s}) - \mathbf{z}$ and $\mathbf{W} = \boldsymbol{\Sigma}^{-1}$ gives exactly the weighted least squares objective. The quantity $\mathbf{r}^T\boldsymbol{\Sigma}^{-1}\mathbf{r}$ is the squared Mahalanobis distance, which is the natural notion of distance when the coordinates have different variances and are correlated, since it measures displacement in units of standard deviation along each principal direction of the covariance.

When measurements are mutually independent their joint negative log likelihood is the sum of the individual ones, so the stacked weight matrix is block diagonal with one block per measurement. This is why `doc/Marginalisation.md` §2 records block diagonality as a property of $\mathbf{W}$, and it is an assumption about the sensors rather than a mathematical necessity.

Whitening exploits the fact that a symmetric positive definite matrix admits a factorisation $\mathbf{W} = \mathbf{W^{T}}^{1/2}\mathbf{W}^{1/2}$, for instance by Cholesky or by the eigendecomposition of `doc/Marginalisation.md` §A.1.3. Defining the whitened residual $\tilde{\mathbf{r}} = \mathbf{W}^{1/2} \mathbf{r}$ and the whitened Jacobian $\tilde{\mathbf{J}} = \mathbf{W}^{1/2}\mathbf{J}$ removes the weight from the objective entirely.

$$\frac{1}{2}\mathbf{r}^T\mathbf{W}\mathbf{r} = \frac{1}{2}((\mathbf{W}^{1/2}\mathbf{r}))^T((\mathbf{W}^{1/2}\mathbf{r})) = \frac{1}{2}\|\tilde{\mathbf{r}}\|^2$$

The name is borrowed from signal processing, where a transformation that renders a coloured noise process uncorrelated with unit variance is said to whiten it. That is precisely the effect here, since the whitened residual has identity covariance by construction.

For the visual residuals of this system the weight is isotropic and reduces to a scalar. Writing $w_H$ for the robustifier weight of §2.5 and $\sigma_\text{obs}$ for the pixel standard deviation held in `vio_obs_std_dev`, the weight and its square root are as follows.

$$\mathbf{W}_{ij} = (\frac{w_H}{\sigma_\text{obs}^2}) \mathbf{I}_{2} \qquad \mathbf{W}_{ij}^{1/2} = (\frac{\sqrt{w_H}}{\sigma_\text{obs}}) \mathbf{I}_2$$

For the inertial residuals the weight is not isotropic, since the preintegration covariance is a full nine by nine matrix that grows with the integration interval, and its square root inverse is applied as a matrix. That operation is `get_sqrt_cov_inv()` used by `ImuBlock::linearizeImu`.

#### Significance

Whitening is not a convenience in this system. It is a precondition for the entire square root formulation, and the reason is stated in §2.6. An orthogonal transformation preserves the Euclidean norm but does not preserve a general weighted norm, so the QR based elimination of §2.11 is only legitimate once the weights have been folded into the rows and the objective has become a plain Euclidean norm. A reader who does not see this connection will not understand why the code multiplies by a square root of a weight rather than by the weight itself.

The contrast between the two solver families in this repository makes the point sharply. The classical path at `src/vi_estimator/ba_base.cpp:112` computes `obs_weight = huber_weight / (obs_std_dev * obs_std_dev)`, which is $\mathbf{W}$ itself, because it accumulates $\mathbf{H} = \mathbf{J}^T\mathbf{W}\mathbf{J}$ directly and never needs a square root. The square root path at `include/basalt/linearization/landmark_block_abs_dynamic.hpp:171` computes `std::sqrt(weight) / options_->obs_std_dev`, which is $\mathbf{W}^{1/2}$, because it retains the Jacobian and must keep the objective in a form an orthogonal matrix can act upon.

A consequence worth recording is that after whitening the residuals are dimensionless. A whitened visual residual of magnitude one corresponds to a one standard deviation reprojection error, which is what allows the Huber threshold `vio_obs_huber_thresh`, whose default is one, to be interpreted as a threshold in standard deviations rather than in pixels.

### 2.3 The Gauss-Newton Method

#### Definition

Gauss-Newton is an iterative method for nonlinear least squares in which the residual, rather than the objective, is linearised. At each iteration the linearised residual defines a quadratic model of the objective which is minimised exactly, and the minimiser is applied as an increment.

#### Derivation

Begin from the objective on the manifold of §2.1 with the weights already absorbed by §2.2.

$$E(\xi) = \frac{1}{2} (\|\mathbf{r}(\bar{\mathbf{s}}\oplus\xi))\|^2$$

Expand the residual to first order about $\xi = 0$, which is the current estimate.

$$\mathbf{r}(\bar{\mathbf{s}}\oplus\xi) \approx \mathbf{r} + \mathbf{J}\xi, \qquad \mathbf{r} = \mathbf{r}(\bar{\mathbf{s}}), \qquad \mathbf{J} = (.\frac{\partial \mathbf{r}}{\partial\xi})|_{\xi=0}$$

Substituting gives the quadratic model, which is written $L(\xi)$ throughout this document and in the source comments.

$$L(\xi) = \frac{1}{2}(\|\mathbf{r} + \mathbf{J}\xi)\|^2 = \frac{1}{2}\mathbf{r}^T\mathbf{r} + \xi^T\mathbf{J}^T\mathbf{r} + \frac{1}{2}\xi^T\mathbf{J}^T\mathbf{J}\xi$$

The middle term follows because $\mathbf{r}^T\mathbf{J}\xi$ is a scalar and therefore equal to its own transpose. Differentiating with respect to $\xi$ and setting the gradient to zero gives the normal equations.

$$\frac{\partial L}{\partial \xi} = \mathbf{J}^T\mathbf{r} + \mathbf{J}^T\mathbf{J}\xi = \mathbf{0} \quad\Longrightarrow\quad \mathbf{J}^T\mathbf{J}\,\xi = -\mathbf{J}^T\mathbf{r}$$

Defining $\mathbf{H} = \mathbf{J}^T\mathbf{J}$ and $\mathbf{b} = -\mathbf{J}^T\mathbf{r}$ recovers the form $\mathbf{H}\xi = \mathbf{b}$ used throughout, and derived at greater length in `doc/Marginalisation.md` §2.1.

It is instructive to compare this with Newton's method applied to the same objective, since the difference explains both the strength and the weakness of Gauss-Newton. The exact Hessian of $E$ is obtained by differentiating twice, and contains a second term involving the curvature of the residual function itself.

$$\nabla^2 E = \mathbf{J}^T\mathbf{J} + \sum_i r_i \nabla^2 r_i$$

Gauss-Newton discards the second term. The approximation is good in two circumstances, namely when the residuals $r_i$ are small at the solution, which is the case for a well tracked feature in a correctly calibrated system, and when the residual functions are nearly affine, so that $\nabla^2 r_i$ is itself small. Both hold reasonably well for reprojection residuals near convergence, which is why the method is the standard choice in bundle adjustment [[8]](#bib-8).

The discarded term also explains the failure modes. When the residuals are large at the solution, which happens in the presence of gross outliers or a badly wrong initialisation, the approximation degrades and convergence can be slow or absent. Gauss-Newton converges quadratically only in the zero residual limit and linearly otherwise, with a rate that worsens as the residual grows. The damping of §2.4 is the standard remedy.

A second property of $\mathbf{H} = \mathbf{J}^T\mathbf{J}$ is that it is positive semi-definite by construction, since $\mathbf{v}^T\mathbf{J}^T\mathbf{J}\mathbf{v} = \|\mathbf{J}\mathbf{v}\|^2 \geq 0$ for every $\mathbf{v}$. It is positive definite if and only if $\mathbf{J}$ has full column rank. The step is therefore always a descent direction when it exists, but it may fail to exist when $\mathbf{J}$ is rank deficient, which in this system is not a pathology but the normal condition, for the reasons given in §2.15.

#### Significance

Three uses of this material appear directly in the implementation.

The normal equations are what `get_dense_H_b` returns, at `src/linearization/linearization_abs_qr.cpp:536-591`, and what the solver factors. Note that the code carries $\mathbf{b} = +\mathbf{J}^T\mathbf{r}$ rather than the negated form, so the caller negates the solved increment at `sqrt_keypoint_vio.cpp:1388`.

The quadratic model $L(\xi)$ is not merely a device for producing the step. Its predicted decrease is computed explicitly and compared against the achieved decrease, and the ratio drives the damping policy of §2.4. The predicted decrease follows from evaluating $L$ at zero and at the step.

$$l_\text{diff} = L(0) - L(\xi) = -\xi^T\mathbf{J}^T\mathbf{r} - \frac{1}{2}\xi^T\mathbf{J}^T\mathbf{J}\xi = -(\mathbf{J}\xi)^T((\mathbf{r} + \frac{1}{2}\mathbf{J}\xi))$$

The final form is the one the code uses, and it is preferred because it needs only the matrix vector product $\mathbf{J}\xi$ and never $\mathbf{J}^T\mathbf{J}$. It is accumulated block by block, in `LandmarkBlockAbsDynamic::backSubstitute` at `landmark_block_abs_dynamic.hpp:294-301`, in `ImuBlock::backSubstitute` at `imu_block.hpp:197-223` and in `computeMargPriorModelCostChange`, and the source states the derivation in a comment at `landmark_block_abs_dynamic.hpp:269-278`.

Finally, the default configuration sets `vio_use_lm = false` at `src/utils/vio_config.cpp:77`. The outer loop is therefore Gauss-Newton with a diagonal regularisation rather than a full Levenberg-Marquardt trust region, although the damping machinery of §2.4 is present and is exercised whenever a step is rejected.

### 2.4 Damping, Trust Regions and the Gain Ratio

#### Definition

Levenberg-Marquardt damping modifies the normal equations by adding a positive multiple of a diagonal matrix, giving $(\mathbf{H} + \lambda\mathbf{D})\xi = \mathbf{b}$, and adapts $\lambda$ between iterations according to how well the quadratic model predicted the observed change in cost.

#### Derivation

The motivation is that the quadratic model is trustworthy only near the current estimate. Two failures of the undamped step are possible. If $\mathbf{H}$ is singular or nearly so, the step is undefined or enormous. If the model is a poor approximation, a step that minimises the model may increase the true objective.

Levenberg's remedy [[14]](#bib-14) is to add $\lambda\mathbf{I}$, which interpolates between two well understood algorithms. As $\lambda \to 0$ the step tends to the Gauss-Newton step, which is fast near a solution. As $\lambda \to \infty$ the system tends to $\lambda\xi = \mathbf{b}$, so the step tends to $\mathbf{b}/\lambda$, which is a short step along the negative gradient, and gradient descent with a small step is reliable if slow. The parameter therefore trades speed against reliability.

Marquardt's refinement [[15]](#bib-15) is to add $\lambda\operatorname{diag}(\mathbf{H})$ rather than $\lambda\mathbf{I}$. The reason is invariance. Under a rescaling of the variables the identity is not transformed consistently, so a fixed $\lambda$ means different things for variables measured in metres and in radians, whereas the diagonal of $\mathbf{H}$ rescales with the variables and preserves the relative treatment of each. This is the variant `basalt` uses, at `sqrt_keypoint_vio.cpp:1358-1361`.

```cpp
VecX Hdiag_lambda = (H.diagonal() * lambda).cwiseMax(min_lambda);
MatX H_copy = H;
H_copy.diagonal() += Hdiag_lambda;
```

The clamp by `min_lambda` guarantees that the added term is strictly positive even where a diagonal entry of $\mathbf{H}$ vanishes, which makes the damped matrix strictly positive definite and therefore invertible regardless of the rank of $\mathbf{H}$. This is the mechanism by which the rank deficiency of §2.15 is tolerated without any explicit gauge fixing in the solver.

There is an equivalent trust region reading. The damped step is the exact solution of the model minimisation subject to a bound on the step length, in the metric defined by $\mathbf{D}$, and $\lambda$ is the Lagrange multiplier of that constraint [[10]](#bib-10). Increasing $\lambda$ shrinks the implied trust radius.

$$\min_\xi L(\xi) \quad\text{subject to}\quad \xi^T\mathbf{D}\xi \leq \Delta^2$$

The adaptation of $\lambda$ is governed by the gain ratio, which compares the decrease actually achieved against the decrease the model predicted.

$$\rho = \frac{E(\mathbf{0}) - E(\xi)}{L(\mathbf{0}) - L(\xi)} = \frac{f_\text{diff}}{l_\text{diff}}$$

A ratio near one indicates that the model is accurate and the damping may be relaxed. A ratio near zero or negative indicates that the model is unreliable and the damping must be increased with the step rejected. `basalt` implements the update rule of Nielsen [[16]](#bib-16), at `sqrt_keypoint_vio.cpp:1492-1558`.

```cpp
lambda *= std::max<Scalar>(
    Scalar(1.0) / 3,
    1 - std::pow<Scalar>(2 * relative_decrease - 1, 3));
lambda = std::max(min_lambda, lambda);
lambda_vee = initial_vee;
```

```cpp
lambda = lambda_vee * lambda;
lambda_vee *= vee_factor;
restore();
```

On acceptance the cubic rule shrinks $\lambda$ by a factor that approaches one third when the gain ratio approaches one, and shrinks it less as the ratio degrades, and the auxiliary factor is reset. On rejection $\lambda$ is multiplied by that auxiliary factor which is itself then doubled, so that consecutive rejections escalate geometrically rather than linearly. Both constants are two, declared at `include/basalt/vi_estimator/sqrt_keypoint_vio.h:264-266`.

#### Significance

Two aspects bear directly on the implementation described later.

The first is that damping can be expressed without ever forming $\mathbf{H}$, by appending rows to the Jacobian, which is what makes it compatible with the square root formulation. Consider augmenting the system as follows.

$$(\|\begin{bmatrix}\mathbf{J}\\ \sqrt{\lambda}\,\mathbf{D}^{1/2}\end{bmatrix}\xi + \begin{bmatrix}\mathbf{r}\\ \mathbf{0}\end{bmatrix})\|^2 = \|\mathbf{J}\xi + \mathbf{r}\|^2 + \lambda\,\xi^T\mathbf{D}\xi$$

The normal equations of the augmented system are $(\mathbf{J}^T\mathbf{J} + \lambda\mathbf{D})\xi = -\mathbf{J}^T\mathbf{r}$, which is the damped system. Damping in square root form is therefore the addition of rows carrying $\sqrt{\lambda}$ on the diagonal, and this is why `setPoseDamping` precomputes `pose_damping_diagonal_sqrt` and why the three reserved damping rows exist in every landmark block. §3.1.7 describes the folding of those rows into an existing triangular factor.

The second is that two damping mechanisms coexist in this codebase and only one of them runs. Landmark damping is applied in square root form inside each landmark block, at `landmark_block_abs_dynamic.hpp:219-259`, and is live. Pose damping in square root form exists at `linearization_abs_qr.cpp:593-602` but its call site is commented out at `sqrt_keypoint_vio.cpp:1314-1319`, so the pose damping that actually executes is the dense diagonal modification quoted above, applied after the normal equations have been formed. A reader comparing the two will find that the square root version writes only $N_\text{cameras}\times 6$ diagonal entries whereas the dense version touches the full diagonal including velocity and bias, and the discrepancy is invisible in practice only because the former is dead.

### 2.5 Robust Cost Functions and Iteratively Reweighted Least Squares

#### Definition

A robust cost function replaces the square of the residual norm by a function that grows more slowly for large residuals, so that a small number of grossly wrong measurements cannot dominate the objective. The Huber function [[9]](#bib-9) is quadratic within a threshold and linear beyond it.

#### Derivation

The need arises because least squares is the maximum likelihood estimator only under a Gaussian noise model, and feature correspondences are not Gaussian. A mismatched feature produces a residual of arbitrary magnitude, and since its contribution to a quadratic objective grows without bound it can displace the estimate arbitrarily far. A single outlier among thousands of good measurements can therefore ruin an estimate.

The Huber cost with threshold $\delta$ is defined piecewise, and is constructed so that the value and the first derivative are continuous at the transition.

$$\rho_\delta(e) = \begin{cases}\frac{1}{2}e^2 & |e| \leq \delta\\[2pt] \delta((|e| - \frac{1}{2}\delta)) & |e| > \delta\end{cases}$$

Minimising a sum of $\rho_\delta(\|\mathbf{r}_i\|)$ is not a least squares problem, but it can be turned into a sequence of them. Differentiating the robust objective with respect to the state gives the following, where the chain rule has been applied through the residual norm.

$$\frac{\partial}{\partial\xi}\sum_i \rho_\delta(\|\mathbf{r}_i\|) = \sum_i \frac{\rho'_\delta(\|\mathbf{r}_i\|)}{\|\mathbf{r}_i\|}\,\mathbf{r}_i^T\mathbf{J}_i$$

Defining the weight $w_i = \rho'_\delta(\|\mathbf{r}_i\|)/\|\mathbf{r}_i\|$ makes this identical to the gradient of an ordinary weighted least squares problem with that weight held fixed. This is the method of iteratively reweighted least squares, in which weights are recomputed from the current residuals at each iteration and the resulting weighted problem is solved as though the weights were constants. For the Huber function the weight is one within the threshold and decays as the reciprocal of the residual norm beyond it.

$$w_H(\mathbf{r}) = \begin{cases}1 & \|\mathbf{r}\| \leq \delta\\[2pt] \dfrac{\delta}{\|\mathbf{r}\|} & \|\mathbf{r}\| > \delta\end{cases}$$

The implementation is at `landmark_block_abs_dynamic.hpp:474-491`, and returns both the weight and the corresponding robustified error value.

```cpp
inline std::tuple<Scalar, Scalar> compute_error_weight(
    Scalar res_squared) const {
  // Note: Definition of cost is 0.5 ||r(x)||^2 to be in line with ceres

  if (options_->huber_parameter > 0) {
    // use huber norm
    const Scalar huber_weight =
        res_squared <= options_->huber_parameter * options_->huber_parameter
            ? Scalar(1)
            : options_->huber_parameter / std::sqrt(res_squared);
    const Scalar error =
        Scalar(0.5) * (2 - huber_weight) * huber_weight * res_squared;
    return {error, huber_weight};
  } else {
    // use squared norm
    return {Scalar(0.5) * res_squared, Scalar(1)};
  }
}
```

The error expression deserves a moment. Substituting $w_H = \delta/\|\mathbf{r}\|$ into $\frac{1}{2}(2-w_H)w_H\|\mathbf{r}\|^2$ gives $\delta\|\mathbf{r}\| - \frac{1}{2}\delta^2$, which is exactly the linear branch of the Huber cost, and substituting $w_H = 1$ gives $\frac{1}{2}\|\mathbf{r}\|^2$, the quadratic branch. The single expression therefore reproduces both branches, which is why the code needs no conditional when computing the error.

The weight enters the system through its square root, because the square root formulation requires the whitening of §2.2 rather than the weight itself. The combined scaling applied to each residual row is the following, and it is a single scalar multiplication.

$$s = \frac{\sqrt{w_H}}{\sigma_\text{obs}}$$

A subtlety worth recording is that this treatment is the standard approximation rather than the exact Newton step for a robust objective. The exact Gauss-Newton Hessian of a robustified cost contains an additional term arising from the derivative of the weight with respect to the residual, and neglecting it is equivalent to treating the weight as constant during the step. Triggs and colleagues [[8]](#bib-8) discuss the correction and the circumstances under which it matters. Ceres and `basalt` alike use the simple form, which is well behaved because the neglected term is negative semi-definite for a convex robustifier and its omission therefore tends to produce conservative steps rather than divergent ones.

#### Significance

Three practical points follow.

The threshold is expressed in whitened units. Because the whitening of §2.2 divides by $\sigma_\text{obs}$ before the Huber test is applied, a threshold of one, which is the default `vio_obs_huber_thresh`, corresponds to one standard deviation of pixel noise and not to one pixel. Changing `vio_obs_std_dev` therefore silently changes the effective robustifier threshold in pixels, which is a coupling a tuner must be aware of.

Huber is a soft rejection rather than a hard one. It down weights an outlier without removing it, so a grossly wrong observation still contributes. Hard rejection is handled separately by outlier filtering outside the linearisation.

The weights are recomputed at every call to `linearizeProblem`, since they depend on the current residuals. They are therefore constant within one linearisation but vary between outer iterations, which is exactly the iteratively reweighted structure described above.

### 2.6 Orthogonal Matrices and Invariance of the Euclidean Norm

#### Definition

A real square matrix $\mathbf{Q}$ is orthogonal when $\mathbf{Q}^T\mathbf{Q} = \mathbf{Q}\mathbf{Q}^T = \mathbf{I}$, equivalently when its columns form an orthonormal basis, equivalently when $\mathbf{Q}^{-1} = \mathbf{Q}^T$.

#### Derivation

The property that matters is invariance of the Euclidean norm. For any vector $\mathbf{v}$ the following holds by direct computation.

$$(\|\mathbf{Q}^T\mathbf{v})\|^2 = ((\mathbf{Q}^T\mathbf{v}))^T((\mathbf{Q}^T\mathbf{v})) = \mathbf{v}^T\mathbf{Q}\mathbf{Q}^T\mathbf{v} = \mathbf{v}^T\mathbf{v} = \|\mathbf{v}\|^2$$

Geometrically an orthogonal transformation is a rigid motion of the space, being a composition of rotations and reflections, and it therefore preserves lengths and angles. Two further consequences are used later. The singular values of a matrix are unchanged by orthogonal multiplication on either side, since $(\mathbf{Q}\mathbf{A})^T(\mathbf{Q}\mathbf{A}) = \mathbf{A}^T\mathbf{A}$, from which the condition number of §2.13 is also unchanged. And the product of orthogonal matrices is orthogonal, so a sequence of elementary orthogonal transformations composes into one.

Applying this to the least squares objective gives the fact on which the whole square root method rests. For any orthogonal $\mathbf{Q}$ of the appropriate size the following two minimisations are the same problem, with the same minimiser and the same minimum value.

$$\min_\xi(\|\mathbf{J}\xi + \mathbf{r})\|^2 = \min_\xi(\|\mathbf{Q}^T((\mathbf{J}\xi + \mathbf{r})))\|^2 = \min_\xi(\|\mathbf{Q}^T\mathbf{J}\,\xi + \mathbf{Q}^T\mathbf{r})\|^2$$

#### Significance

This is the licence to transform. It says that the linearised system may be multiplied on the left by any orthogonal matrix whatever, and the answer will not change. A judicious choice of $\mathbf{Q}$ can therefore introduce zeros into the Jacobian at no cost in information whatever, and that is precisely how landmarks are eliminated in §2.11.

Two qualifications should be stated because both are load bearing.

The transformation must be applied to the residual as well as to the Jacobian, and to every column of the Jacobian simultaneously. Applying it to a submatrix alone would not be an identity on the objective. This is exactly why the landmark block of §3.1.5 stores the pose Jacobian, the landmark Jacobian and the residual in one matrix, since a single call can then transform all three in lock step.

The invariance holds for the Euclidean norm and not for a weighted norm, because $\|\mathbf{Q}^T\mathbf{v}\|_\mathbf{W}^2 = \mathbf{v}^T\mathbf{Q}\mathbf{W}\mathbf{Q}^T\mathbf{v}$, which differs from $\mathbf{v}^T\mathbf{W}\mathbf{v}$ unless $\mathbf{Q}$ commutes with $\mathbf{W}$. This is the formal reason the whitening of §2.2 must precede any orthogonal transformation, and it is the single most important dependency in this section.

Finally, the invariance of singular values under orthogonal multiplication is what makes the square root approach numerically attractive rather than merely elegant. Transforming a Jacobian by $\mathbf{Q}^T$ does not worsen its conditioning at all, whereas forming $\mathbf{J}^T\mathbf{J}$ squares it, as §2.13 shows.

### 2.7 QR Decomposition

#### Definition

For a real matrix $\mathbf{A}$ of size $m \times n$ with $m \geq n$, a QR decomposition is a factorisation $\mathbf{A} = \mathbf{Q}\mathbf{R}$ in which $\mathbf{Q}$ is $m \times m$ orthogonal and $\mathbf{R}$ is $m \times n$ upper triangular, meaning that all entries below the main diagonal vanish.

#### Derivation

Because $\mathbf{R}$ is upper triangular and taller than it is wide, its last $m-n$ rows are entirely zero, so the factorisation splits naturally. Partition $\mathbf{Q}$ by columns into the first $n$ and the remaining $m-n$.

$$\mathbf{A} = \mathbf{Q}\mathbf{R} = \begin{bmatrix}\mathbf{Q}_1 & \mathbf{Q}_2\end{bmatrix}\begin{bmatrix}\mathbf{R}_1\\ \mathbf{0}\end{bmatrix} = \mathbf{Q}_1\mathbf{R}_1$$

The right hand form is the reduced or thin decomposition, with $\mathbf{Q}_1$ of size $m\times n$ and $\mathbf{R}_1$ upper triangular of size $n \times n$. The left hand form is the full decomposition, and it is the one this document needs because $\mathbf{Q}_2$ is the object of interest.

Existence is proved by construction. Suppose a sequence of orthogonal transformations $\mathbf{P}_1, \ldots, \mathbf{P}_n$ can be chosen so that $\mathbf{P}_k$ annihilates the entries below the diagonal in column $k$ while leaving columns $1$ through $k-1$ undisturbed. Then their composition triangularises the matrix, and since each is orthogonal so is the product.

$$\mathbf{P}_n\cdots\mathbf{P}_1\mathbf{A} = \mathbf{R} \quad\Longrightarrow\quad \mathbf{A} = ((\mathbf{P}_n\cdots\mathbf{P}_1))^T\mathbf{R} = \mathbf{Q}\mathbf{R}$$

Three constructions of the $\mathbf{P}_k$ are standard and their differences matter.

Gram-Schmidt orthonormalisation builds $\mathbf{Q}_1$ column by column by subtracting from each column of $\mathbf{A}$ its projections onto the previously computed columns. It is the most intuitive derivation and the worst in floating point, because rounding causes the computed columns to lose orthogonality progressively, and the modified variant improves but does not cure this. It is not used here and is mentioned only so that a reader who knows QR from a linear algebra course understands why the code looks nothing like it.

Householder reflections, treated in §2.8, annihilate an entire column below the diagonal with one transformation. They are backward stable and are the default in this codebase.

Givens rotations, treated in §2.9, annihilate one entry at a time. They are equally stable and are preferred when the matrix is already nearly triangular, so that only a few entries need removing.

The orthogonality of $\mathbf{Q}$ gives four relations among the blocks, each of which is used later.

$$\mathbf{Q}_1^T\mathbf{Q}_1 = \mathbf{I}_n, \qquad \mathbf{Q}_2^T\mathbf{Q}_2 = \mathbf{I}_{m-n}, \qquad \mathbf{Q}_1^T\mathbf{Q}_2 = \mathbf{0}, \qquad \mathbf{Q}_1\mathbf{Q}_1^T + \mathbf{Q}_2\mathbf{Q}_2^T = \mathbf{I}_m$$

The first two express orthonormality within each block, the third mutual orthogonality between blocks, and the fourth is the resolution of the identity, which states that the two blocks together span the whole space and their projectors sum to the identity. This fourth relation, usually written in the rearranged form $\mathbf{Q}_1\mathbf{Q}_1^T = \mathbf{I} - \mathbf{Q}_2\mathbf{Q}_2^T$, is the key to the equivalence proof of §2.11.

The property that gives the method its name in this context follows immediately. Since $\mathbf{A} = \mathbf{Q}_1\mathbf{R}_1$, the third relation gives the following.

$$\mathbf{Q}_2^T\mathbf{A} = \mathbf{Q}_2^T\mathbf{Q}_1\mathbf{R}_1 = \mathbf{0}$$

The columns of $\mathbf{Q}_2$ therefore span the left nullspace of $\mathbf{A}$, which is the orthogonal complement of the column space of $\mathbf{A}$ in $\mathbb{R}^m$. Its dimension is $m - n$ when $\mathbf{A}$ has full column rank, and larger when it does not.

Two further facts are needed. The triangular factor is a square root of the corresponding information matrix, which follows in one line.

$$\mathbf{R}_1^T\mathbf{R}_1 = \mathbf{R}_1^T\mathbf{Q}_1^T\mathbf{Q}_1\mathbf{R}_1 = ((\mathbf{Q}_1\mathbf{R}_1))^T((\mathbf{Q}_1\mathbf{R}_1)) = \mathbf{A}^T\mathbf{A}$$

And the cost, for a dense $m\times n$ matrix with $m \geq n$, is $2mn^2 - \frac{2}{3}n^3$ floating point operations by Householder reflections, roughly twice the cost of forming and factoring the normal equations, which is the price paid for the improved conditioning of §2.13 [[7]](#bib-7).

#### Significance

The left nullspace is the reason QR appears in this codebase at all. If $\mathbf{A}$ is taken to be the landmark Jacobian $\mathbf{J}_l$ of a single landmark, then left multiplying the whole linear system by $\mathbf{Q}_2^T$ produces a system in which that landmark does not appear, because $\mathbf{Q}_2^T\mathbf{J}_l = \mathbf{0}$ identically. By §2.6 the transformation changes nothing about the problem. The landmark has been eliminated by projection rather than by inversion, and §2.11 develops this into the full method.

The relation $\mathbf{R}_1^T\mathbf{R}_1 = \mathbf{A}^T\mathbf{A}$ is the reason the marginalisation routine of §3.3.3 needs no factorisation step at its end. Having run a QR on the stacked Jacobian, it already holds a square root of the new prior's information matrix, and can store the triangular factor directly.

For a landmark the numbers are small and worth having in mind. The landmark Jacobian has three columns and two rows per observation, so a landmark with eight observations gives a sixteen by three matrix, whose QR costs a few hundred operations and whose $\mathbf{Q}_2$ has thirteen columns. The elimination is therefore cheap, and the expensive part of the pipeline is the assembly rather than the decomposition.

The definitive reference for the numerical treatment is Golub and Van Loan [[7]](#bib-7), and for least squares problems specifically Björck [[11]](#bib-11). The application to bundle adjustment in the form used here is due to Demmel and colleagues [[1]](#bib-1), with the sliding window extension in [[2]](#bib-2).

### 2.8 Householder Reflections

#### Definition

A Householder reflection is the orthogonal matrix determined by a nonzero vector $\mathbf{v}$, which reflects any vector through the hyperplane whose normal is $\mathbf{v}$.

$$\mathbf{P} = \mathbf{I} - \tau\,\mathbf{v}\mathbf{v}^T, \qquad \tau = \frac{2}{\mathbf{v}^T\mathbf{v}}$$

#### Derivation

That $\mathbf{P}$ is symmetric is immediate from the form, and that it is orthogonal follows by expansion using $\tau\,\mathbf{v}^T\mathbf{v} = 2$.

$$\mathbf{P}^T\mathbf{P} = \mathbf{P}^2 = \mathbf{I} - 2\tau\mathbf{v}\mathbf{v}^T + \tau^2\mathbf{v}((\mathbf{v}^T\mathbf{v}))\mathbf{v}^T = \mathbf{I} - 2\tau\mathbf{v}\mathbf{v}^T + 2\tau\mathbf{v}\mathbf{v}^T = \mathbf{I}$$

Being symmetric and orthogonal it is its own inverse, which is the algebraic signature of a reflection.

The construction that matters is the choice of $\mathbf{v}$ that maps a given vector onto a multiple of the first coordinate axis. Given $\mathbf{x} \in \mathbb{R}^m$, the requirement is $\mathbf{P}\mathbf{x} = \beta\mathbf{e}_1$ for some scalar $\beta$. Since a reflection preserves length, $|\beta| = \|\mathbf{x}\|$. Taking $\mathbf{v} = \mathbf{x} - \beta\mathbf{e}_1$ and substituting confirms the construction works, and the two admissible signs of $\beta$ are both valid mathematically.

$$\mathbf{v} = \mathbf{x} + \operatorname{sign}(x_1)\|\mathbf{x}\|\,\mathbf{e}_1 \quad\Longrightarrow\quad \mathbf{P}\mathbf{x} = -\operatorname{sign}(x_1)\|\mathbf{x}\|\,\mathbf{e}_1$$

The sign is chosen to match that of the leading entry rather than to oppose it, and the reason is numerical. The alternative choice makes the first component of $\mathbf{v}$ equal to $x_1 - \|\mathbf{x}\|$, and when $\mathbf{x}$ is already nearly a multiple of $\mathbf{e}_1$ these two quantities are nearly equal, so their difference suffers catastrophic cancellation and $\mathbf{v}$ is computed with large relative error. The sign chosen above makes the subtraction an addition and removes the cancellation entirely. This single detail is the difference between a stable and an unstable implementation, and it is the reason one uses a library routine such as Eigen's `makeHouseholder` rather than writing the formula out.

The reflection is never formed as a matrix. Its action on a matrix $\mathbf{B}$ is computed as a matrix vector product followed by a rank one update, which for an $m \times k$ target costs $4mk$ operations instead of the $2m^2k$ a dense product would cost.

$$\mathbf{P}\mathbf{B} = \mathbf{B} - \tau\,\mathbf{v}((\mathbf{v}^T\mathbf{B}))$$

A full QR is obtained by applying such a reflection to each column in turn, where the $k$-th acts only on rows $k$ and below so that the already triangularised columns are untouched. Householder QR is backward stable, meaning the computed factors are the exact factors of a matrix within a small multiple of machine precision of the original, and the computed $\mathbf{Q}$ is orthogonal to that same accuracy [[7]](#bib-7).

#### Significance

Householder reflections are the default in `basalt`, selected by `LandmarkBlock::Options::use_householder`, and the source cites the standard argument that reflections beat rotations for dense matrices because each annihilates a whole column at once. The implementation at `landmark_block_abs_dynamic.hpp:456-472` calls Eigen's `makeHouseholder` and `applyHouseholderOnTheLeft` directly rather than constructing an `Eigen::HouseholderQR` object, and there are two reasons for that choice. The reflection must be applied across every column of the storage matrix rather than to the landmark columns alone, which the object oriented interface does not express naturally, and the orthogonal factor is never needed in explicit form, so building it would be waste.

The same primitive appears again at a larger scale in `MargHelper::marginalizeHelperSqrtToSqrt`, where a hand written rank revealing Householder QR eliminates frame variables rather than landmark parameters, as §3.3.3 describes.

For historical completeness, the transformation is due to Householder [[12]](#bib-12).

### 2.9 Givens Rotations

#### Definition

A Givens rotation is the orthogonal matrix that equals the identity except in the four entries at the intersections of rows and columns $i$ and $j$, where it holds a two dimensional rotation.

$$\mathbf{G}(i,j,\theta) = \begin{bmatrix}\ddots & & & \\ & c & s & \\ & -s & c & \\ & & & \ddots\end{bmatrix}, \qquad c^2 + s^2 = 1$$

#### Derivation

Applied on the left, such a matrix modifies only rows $i$ and $j$ and leaves every other row untouched. Its effect on the two affected entries of a column is a planar rotation, and the parameters can be chosen to annihilate one of them. Given the pair $(a, b)$ occupying rows $i$ and $j$ of some column, the choice below sends it to $(\sqrt{a^2+b^2},\,0)$.

$$c = \frac{a}{\sqrt{a^2+b^2}}, \qquad s = \frac{b}{\sqrt{a^2+b^2}}$$

A robust implementation computes the hypotenuse by scaling to avoid overflow, and Eigen's `makeGivens` does so. A full QR follows by sweeping such rotations up each column, the algorithm being 5.2.4 of Golub and Van Loan [[7]](#bib-7), which the source at `landmark_block_abs_dynamic.hpp:444-454` cites by page number.

The cost of a Givens QR of a dense matrix is about fifty per cent higher than the Householder equivalent, because annihilating a column of length $m$ requires $m-1$ rotations each touching two rows, rather than one reflection touching all rows. For a dense matrix Householder is therefore preferred. The calculus reverses when the matrix is already nearly triangular, since then only a handful of entries are nonzero below the diagonal and a Givens sweep touches only those, whereas a Householder reflection would still operate on full columns.

The property that decides the matter in this codebase is invertibility of an individual rotation. Each $\mathbf{G}$ is orthogonal, so its inverse is its transpose, and applying the transpose to the same row pair undoes the rotation exactly. A sequence of rotations can therefore be recorded and replayed in reverse to restore the original matrix to within rounding error.

#### Significance

`basalt` uses Givens rotations for exactly the case they suit. When Levenberg-Marquardt damping is applied to a landmark, three rows carrying $\sqrt{\lambda}$ must be folded into a factor that is already upper triangular. Only six entries need annihilating, being three in the first column, two in the second and one in the third, so six rotations suffice and a fresh Householder QR would be waste.

More importantly, the solver varies $\lambda$ across the inner backtracking loop and must be able to change the damping without recomputing the decomposition. Storing the six rotations makes this possible, since the undo path replays them in reverse using the adjoint of each. The implementation is at `landmark_block_abs_dynamic.hpp:219-259` and is described in §3.1.7. Householder reflections could in principle be stored and inverted too, but each would act on the whole column rather than on a row pair, so the update would not be local and the saving would vanish.

For historical completeness, the transformation is due to Givens [[13]](#bib-13).

### 2.10 The Schur Complement and Block Elimination

#### Definition

Given a block partitioned matrix, the Schur complement of one diagonal block is the matrix obtained by eliminating the corresponding variables from the linear system by block Gaussian elimination.

$$\mathbf{M} = \begin{bmatrix}\mathbf{A} & \mathbf{B}\\ \mathbf{C} & \mathbf{D}\end{bmatrix} \quad\Longrightarrow\quad \mathbf{M}/\mathbf{D} = \mathbf{A} - \mathbf{B}\mathbf{D}^{-1}\mathbf{C}$$

#### Derivation

Consider the linear system partitioned to match, with $\xi_\alpha$ the variables to retain and $\xi_\beta$ those to eliminate.

$$\begin{bmatrix}\mathbf{H}_{\alpha\alpha} & \mathbf{H}_{\alpha\beta}\\ \mathbf{H}_{\beta\alpha} & \mathbf{H}_{\beta\beta}\end{bmatrix}\begin{bmatrix}\xi_\alpha\\ \xi_\beta\end{bmatrix} = \begin{bmatrix}\mathbf{b}_\alpha\\ \mathbf{b}_\beta\end{bmatrix}$$

The lower block row states $\mathbf{H}_{\beta\alpha}\xi_\alpha + \mathbf{H}_{\beta\beta}\xi_\beta = \mathbf{b}_\beta$, which can be solved for the eliminated variables in terms of the retained ones provided $\mathbf{H}_{\beta\beta}$ is invertible.

$$\xi_\beta = \mathbf{H}_{\beta\beta}^{-1}((\mathbf{b}_\beta - \mathbf{H}_{\beta\alpha}\xi_\alpha))$$

Substituting into the upper block row and collecting terms gives a system in the retained variables alone.

$$((\mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{H}_{\beta\alpha}))\xi_\alpha = \mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{b}_\beta$$

The matrix on the left is the Schur complement $\mathbf{H}^\ast$ and the vector on the right is $\mathbf{b}^\ast$. This derivation is given in `doc/Marginalisation.md` §3.2 and is repeated here only in outline, since the present document needs the properties rather than the derivation.

Four properties are used later. The elimination is exact, in the sense that the reduced system has exactly the same solution for $\xi_\alpha$ as the full system, so no approximation whatever is involved. The Schur complement of a positive definite block within a positive semi-definite matrix is positive semi-definite, which is what permits the square root factorisation of `doc/Marginalisation.md` §A.3. The operation is dense making, since $\mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{H}_{\beta\alpha}$ generally has no zero entries even when its factors are sparse, and this is why the marginalisation prior is a dense matrix. And the elimination requires an inverse, which is the operation the square root formulation exists to avoid.

The structure the Schur complement exploits in bundle adjustment is worth stating explicitly. Order the variables with the frame variables first and the landmarks second. A reprojection residual involves one landmark and at most two frames, so no residual involves two distinct landmarks. Consequently the landmark block of the information matrix has no coupling between different landmarks and is block diagonal with three by three blocks.

$$\mathbf{H} = \begin{bmatrix}\mathbf{H}_{pp} & \mathbf{H}_{pl}\\ \mathbf{H}_{lp} & \mathbf{H}_{ll}\end{bmatrix}, \qquad \mathbf{H}_{ll} = \operatorname{blkdiag}((\mathbf{H}_{l_1l_1}, \ldots, \mathbf{H}_{l_Ll_L}))$$

This is the arrowhead structure described in `doc/VIO.md` §2.2.8. Its consequence is that $\mathbf{H}_{ll}^{-1}$ is obtained by inverting each three by three block independently, at trivial cost, so the whole elimination costs $O(L K^2)$ for $L$ landmarks and $K$ frame variables rather than anything cubic in $L$. Bundle adjustment is tractable at scale entirely because of this structure, and every method in this document, the QR one included, is a way of exploiting it.

#### Significance

The Schur complement appears three times in this system and it is worth separating the roles.

Landmark elimination within one linearisation is performed by the QR method of §2.11 in the default configuration, which is algebraically equivalent to the Schur complement but does not compute it. The two alternative strategies `ABS_SC` and `REL_SC` perform it directly.

Variable elimination during marginalisation is performed by the Schur complement in two of the three `MargHelper` routines, and by QR in the third, as §3.3.3 describes.

The probabilistic interpretation, developed in §2.17, is that the Schur complement is exactly the operation of marginalising a variable out of a Gaussian distribution, which is where the name marginalisation comes from and why the operation is information preserving.

The standard reference on the Schur complement and its properties is Zhang [[24]](#bib-24).

### 2.11 Nullspace Projection and its Equivalence to the Schur Complement

#### Definition

Nullspace projection eliminates a group of variables from a least squares problem by left multiplying the linearised system with $\mathbf{Q}_2^T$, where $\mathbf{Q}_2$ spans the left nullspace of the Jacobian block belonging to those variables.

#### Derivation

Partition the Jacobian by variable group, with $\mathbf{J}_p$ the columns of the frame variables and $\mathbf{J}_l$ the columns of the landmark to be eliminated, and let $\mathbf{J}_l = \mathbf{Q}\mathbf{R}$ be a full QR decomposition with $\mathbf{Q} = [\mathbf{Q}_1\ \mathbf{Q}_2]$ as in §2.7. By the invariance of §2.6 the objective may be multiplied by $\mathbf{Q}^T$ without change.

$$(\|\mathbf{r} + \begin{bmatrix}\mathbf{J}_p & \mathbf{J}_l\end{bmatrix}\begin{bmatrix}\xi_p\\ \xi_l\end{bmatrix})\|^2 = (\|\mathbf{Q}^T\mathbf{r} + \begin{bmatrix}\mathbf{Q}^T\mathbf{J}_p & \mathbf{Q}^T\mathbf{J}_l\end{bmatrix}\begin{bmatrix}\xi_p\\ \xi_l\end{bmatrix})\|^2$$

Now evaluate the transformed blocks. By construction $\mathbf{Q}^T\mathbf{J}_l = [\mathbf{R}_1;\,\mathbf{0}]$, so the landmark columns vanish entirely below the third row. Writing the transformed system out by block row gives the following.

$$\mathbf{Q}^T\begin{bmatrix}\mathbf{J}_l & \mathbf{J}_p & \mathbf{r}\end{bmatrix} = \begin{bmatrix}\mathbf{R}_1 & \mathbf{Q}_1^T\mathbf{J}_p & \mathbf{Q}_1^T\mathbf{r}\\ \mathbf{0} & \mathbf{Q}_2^T\mathbf{J}_p & \mathbf{Q}_2^T\mathbf{r}\end{bmatrix}$$

Because the squared norm of a stacked vector is the sum of the squared norms of its parts, the objective separates into two independent terms.

$$(\|\mathbf{Q}_1^T\mathbf{r} + \mathbf{Q}_1^T\mathbf{J}_p\xi_p + \mathbf{R}_1\xi_l)\|^2 + (\|\mathbf{Q}_2^T\mathbf{r} + \mathbf{Q}_2^T\mathbf{J}_p\xi_p)\|^2$$

The landmark increment appears in the first term only. Whatever value the frame increment takes, the first term can be driven exactly to zero by the corresponding choice of landmark increment, since $\mathbf{R}_1$ is square, upper triangular and invertible for a well constrained landmark.

$$\xi_l^\ast = -\mathbf{R}_1^{-1}((\mathbf{Q}_1^T\mathbf{r} + \mathbf{Q}_1^T\mathbf{J}_p\,\xi_p^\ast))$$

The minimisation over the frame variables therefore reduces to the second term alone, which contains no landmark variable at all.

$$\min_{\xi_p}(\|\mathbf{Q}_2^T\mathbf{r} + \mathbf{Q}_2^T\mathbf{J}_p\,\xi_p)\|^2$$

This pair of equations is the whole method. The second is solved for the frame increment, and the first then recovers the landmark increment by a triangular solve, which is the back substitution of §3.1.10.

It remains to prove that this reduced problem is the same one the Schur complement produces. Substitute $\mathbf{J}_l = \mathbf{Q}_1\mathbf{R}_1$ into the definitions of the information blocks.

$$\mathbf{H}_{pp} = \mathbf{J}_p^T\mathbf{J}_p, \qquad \mathbf{H}_{pl} = \mathbf{J}_p^T\mathbf{Q}_1\mathbf{R}_1, \qquad \mathbf{H}_{ll} = \mathbf{R}_1^T\mathbf{Q}_1^T\mathbf{Q}_1\mathbf{R}_1 = \mathbf{R}_1^T\mathbf{R}_1$$

Now form the Schur complement and simplify, using $\mathbf{H}_{ll}^{-1} = \mathbf{R}_1^{-1}\mathbf{R}_1^{-T}$ and then the resolution of the identity from §2.7.

$$\mathbf{H}_{pl}\mathbf{H}_{ll}^{-1}\mathbf{H}_{lp} = \mathbf{J}_p^T\mathbf{Q}_1\mathbf{R}_1((\mathbf{R}_1^{-1}\mathbf{R}_1^{-T}))\mathbf{R}_1^T\mathbf{Q}_1^T\mathbf{J}_p = \mathbf{J}_p^T\mathbf{Q}_1\mathbf{Q}_1^T\mathbf{J}_p$$

$$\tilde{\mathbf{H}}_{pp} = \mathbf{H}_{pp} - \mathbf{J}_p^T\mathbf{Q}_1\mathbf{Q}_1^T\mathbf{J}_p = \mathbf{J}_p^T((\mathbf{I} - \mathbf{Q}_1\mathbf{Q}_1^T))\mathbf{J}_p = \mathbf{J}_p^T\mathbf{Q}_2\mathbf{Q}_2^T\mathbf{J}_p = ((\mathbf{Q}_2^T\mathbf{J}_p))^T((\mathbf{Q}_2^T\mathbf{J}_p))$$

The same computation on the right hand side gives the companion result.

$$\tilde{\mathbf{b}}_p = \mathbf{J}_p^T\mathbf{Q}_2\mathbf{Q}_2^T\mathbf{r} = ((\mathbf{Q}_2^T\mathbf{J}_p))^T((\mathbf{Q}_2^T\mathbf{r}))$$

The reduced normal equations of the Schur complement are therefore precisely the normal equations of the nullspace projected least squares problem. The two methods produce the same answer in exact arithmetic, and the QR method produces it in factored form. This equivalence is established in Demmel and colleagues [[1]](#bib-1), whose equations fifteen through twenty five this derivation follows.

#### Significance

The equivalence is worth dwelling upon, because it says something stronger than that two algorithms agree. It says that the Schur complement, which appears to require an inverse, is secretly an orthogonal projection, and that the inverse is an artefact of insisting on working with the squared quantity. Once the problem is kept in square root form the inverse disappears.

The practical differences are three.

Numerically the projection is superior. It never forms $\mathbf{H}_{ll}$, so it never squares the conditioning of the landmark block, and it never inverts anything. A landmark that is poorly constrained, for instance one observed from a very short baseline, produces an ill conditioned $\mathbf{H}_{ll}$ whose inverse is inaccurate, whereas its QR simply produces a small diagonal entry in $\mathbf{R}_1$, which is a benign and detectable condition.

Structurally the projection yields a factored result. The output $\mathbf{Q}_2^T\mathbf{J}_p$ is a square root of the reduced information matrix, so if the consumer also wants a square root, as the marginalisation of §3.3 does, no further work is needed.

Computationally the projection is somewhat more expensive, by roughly the factor of two noted in §2.7, and it produces more rows than the Schur complement produces equations. For a landmark with $n_\text{obs}$ observations the reduced system has $2n_\text{obs} - 3$ rows rather than the $K$ equations of the normal equations form.

The naming throughout the code follows this derivation literally. The identifiers `Q2Jp` and `Q2r` are $\mathbf{Q}_2^T\mathbf{J}_p$ and $\mathbf{Q}_2^T\mathbf{r}$, and `Q1Jl`, `Q1Jp` and `Q1Jr` are the corresponding retained rows. A reader without this derivation will find those names impenetrable, which is the principal reason this section exists.

The terminal equation also appears in `doc/VIO.md` §2.2.8, stated without derivation, and the present section supplies what that section omits.

### 2.12 Cholesky and LDLT Factorisation

#### Definition

A Cholesky factorisation expresses a symmetric positive definite matrix as $\mathbf{H} = \mathbf{L}\mathbf{L}^T$ with $\mathbf{L}$ lower triangular with positive diagonal. The LDLT variant expresses it as $\mathbf{H} = \mathbf{L}\mathbf{D}\mathbf{L}^T$ with $\mathbf{L}$ unit lower triangular and $\mathbf{D}$ diagonal, and in pivoted form as $\mathbf{H} = \mathbf{P}^T\mathbf{L}\mathbf{D}\mathbf{L}^T\mathbf{P}$ for a permutation $\mathbf{P}$.

#### Derivation

Existence for a positive definite matrix follows by induction on the leading entry, and the factorisation is unique under the positivity condition on the diagonal. The cost is $\frac{1}{3}n^3$ operations, half that of a general LU factorisation, the saving coming from symmetry.

The LDLT variant is preferred here for two reasons. It avoids the square roots that plain Cholesky requires on the diagonal, which is a minor efficiency gain and a significant robustness gain, since a square root of a marginally negative quantity is a hard failure whereas a negative entry of $\mathbf{D}$ is merely a signal. And with symmetric pivoting it remains usable for matrices that are positive semi-definite or mildly indefinite, which is exactly the condition the information matrices of §2.15 are in.

Once a factorisation is available, a linear system is solved by two triangular substitutions and a diagonal scaling, at cost $O(n^2)$, which is why the factorisation is computed once and reused when several right hand sides are needed.

The connection to square roots is what this document needs. Given the pivoted factorisation, define the following matrix.

$$\mathbf{J}_\text{marg} = \mathbf{D}^{1/2}\mathbf{L}^T\mathbf{P}$$

Then $\mathbf{J}_\text{marg}^T\mathbf{J}_\text{marg} = \mathbf{P}^T\mathbf{L}\mathbf{D}^{1/2}\mathbf{D}^{1/2}\mathbf{L}^T\mathbf{P} = \mathbf{P}^T\mathbf{L}\mathbf{D}\mathbf{L}^T\mathbf{P} = \mathbf{H}$, so this is a square root factor in the sense of `doc/Marginalisation.md` §A.1.3. It is not the same square root the eigendecomposition of that appendix produces, and it need not be, since a square root is not unique. Any two differ by an orthogonal factor on the left, and by §2.6 that difference is invisible to the objective.

The matching residual is obtained by requiring $\mathbf{J}_\text{marg}^T\mathbf{r}_\text{marg} = \mathbf{b}$, which is a triangular system.

$$\mathbf{r}_\text{marg} = \mathbf{D}^{-1/2}\mathbf{L}^{-1}\mathbf{P}\,\mathbf{b}$$

#### Significance

Both uses appear in the implementation.

The odometry solve factors the damped dense system with `Eigen::LDLT` at `sqrt_keypoint_vio.cpp:1363`. The damping of §2.4 guarantees strict positive definiteness, so the factorisation is safe despite the rank deficiency of the undamped matrix.

The conversion routine `MargHelper::marginalizeHelperSqToSqrt` at `marg_helper.cpp:211-242` uses exactly the construction above to turn a dense Schur complement into a square root prior, and its source comment states the algebra in the same terms.

Two guards in that routine deserve note because they encode the semi-definite reality of §2.15. Entries of $\mathbf{D}$ that come out negative, which can only happen through rounding since the true matrix is positive semi-definite, are clamped to zero before the square root by `ldlt.vectorD().array().max(0).sqrt()`. And components of the residual whose diagonal entry is at or below the smallest representable positive number are set to zero rather than divided, which is the correct handling of a direction carrying no information.

### 2.13 Condition Number, Numerical Stability and the Cost of the Normal Equations

#### Definition

The condition number of a matrix in the two norm is the ratio of its largest to its smallest singular value, and it bounds how much a relative perturbation of the data can be amplified in the solution.

$$\kappa(\mathbf{A}) = \frac{\sigma_\text{max}(\mathbf{A})}{\sigma_\text{min}(\mathbf{A})}$$

#### Derivation

The bound that gives the quantity its meaning is the following. If $\mathbf{A}\mathbf{x} = \mathbf{c}$ and the data is perturbed to $\mathbf{c} + \delta\mathbf{c}$, then the relative change in the solution satisfies the inequality below, and the bound is attained for the worst case perturbation.

$$\frac{\|\delta\mathbf{x}\|}{\|\mathbf{x}\|} \leq \kappa(\mathbf{A})\,\frac{\|\delta\mathbf{c}\|}{\|\mathbf{c}\|}$$

Since floating point arithmetic introduces relative perturbations of order the machine epsilon $\varepsilon$ at every operation, and since a backward stable algorithm returns the exact solution of a nearby problem, the practical rule is that the computed solution has relative error of order $\kappa\varepsilon$. Expressed in decimal digits, roughly $\log_{10}\kappa$ digits are lost.

The central fact for this document is what squaring does. If $\mathbf{A}$ has singular value decomposition $\mathbf{A} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^T$ then $\mathbf{A}^T\mathbf{A} = \mathbf{V}\boldsymbol{\Sigma}^2\mathbf{V}^T$, so the eigenvalues of the product are the squares of the singular values of the factor.

$$\kappa((\mathbf{A}^T\mathbf{A})) = \frac{\sigma_\text{max}^2}{\sigma_\text{min}^2} = \kappa(\mathbf{A})^2$$

Forming the normal equations therefore doubles the number of digits lost. The arithmetic is worth doing concretely. Single precision carries about seven decimal digits and double precision about sixteen. A Jacobian conditioned at $10^4$, which is entirely ordinary for a visual-inertial problem mixing metres, radians and inverse metres, gives an information matrix conditioned at $10^8$. In single precision that consumes the entire budget and the computed solution has no correct digits whatever, whereas the same problem solved through the Jacobian retains three. In double precision the squared problem retains eight digits, which is adequate, and this is why the practice is tolerable at all.

The precision this pipeline actually runs in should be stated plainly, because it bounds how much the argument above matters in practice. The build system enables both `BASALT_INSTANTIATIONS_DOUBLE` and `BASALT_INSTANTIATIONS_FLOAT`, so every linearisation class is compiled for both scalar types. The single precision instantiation of the estimator factory is nonetheless commented out at `src/vi_estimator/vio_estimator.cpp:184-189`, and every call site in the repository, including `controller.cpp`, `vio.cpp`, `rs_t265_vio.cpp` and `vio_sim.cpp`, requests `getVioEstimator<double>`. The shipped pipeline therefore runs in double precision throughout, and the single precision failure described above is not exercised by any current code path. The square root formulation is consequently insurance rather than necessity as the system stands, and its value would become acute if the estimator were ever moved to single precision for an embedded target, which is precisely the setting Demmel and colleagues [[1]](#bib-1) target.

A second, subtler effect is loss of information at formation time. If $\sigma_\text{min}/\sigma_\text{max} < \sqrt{\varepsilon}$ then the smallest singular value is lost entirely in the rounding of the product, and the computed $\mathbf{A}^T\mathbf{A}$ is numerically singular even though $\mathbf{A}$ has full rank. The threshold for this is $\kappa > 10^{3.5}$ in single precision and $\kappa > 10^{8}$ in double.

#### Significance

This is the argument that motivates the square root formulation, and both Demmel papers [[1]](#bib-1) [[2]](#bib-2) state it as the central drawback of the Schur complement approach, the second phrasing it as the condition number of the Hessian being squared compared to the Jacobian.

It also explains a design choice in this codebase that would otherwise appear inconsistent. The odometry path does form the normal equations, calling `get_dense_H_b` and factoring with LDLT at `sqrt_keypoint_vio.cpp:1363`, so it pays the squaring penalty in full. The marginalisation path does not, taking the square root exit `get_dense_Q2Jp_Q2r` and carrying the factored form all the way into the stored prior. The asymmetry is deliberate and well judged. The odometry system is rebuilt from scratch at the next frame, so its conditioning damage is discarded with it and never compounds. The marginalisation prior persists for the entire run and is repeatedly re-marginalised, so any error introduced into it accumulates over thousands of frames. Precision is spent where it accumulates and economised where it does not.

Four further devices in the code exist to control conditioning, and each is a partial remedy rather than a cure.

Column scaling, treated in §2.14, equalises the diagonal and removes the component of ill conditioning that comes purely from inconsistent units.

Damping, treated in §2.4, adds a strictly positive quantity to the diagonal and thereby bounds the smallest eigenvalue away from zero, which caps the condition number at roughly $\sigma_\text{max}^2/\lambda$.

Rank revealing decomposition, used in `marginalizeHelperSqrtToSqrt`, detects directions carrying no information rather than attempting to invert them, using the threshold $\sqrt{\varepsilon}$ which is the standard choice because it is the point below which a singular value is indistinguishable from rounding noise.

Pseudoinversion by complete orthogonal decomposition, used in the two Schur complement routines, handles a rank deficient block gracefully where a plain inverse would fail. The source records the experimental history behind that choice and explicitly warns against a singular value based alternative on accuracy grounds.

### 2.14 Column Scaling and Preconditioning

#### Definition

Column scaling, also called Jacobi preconditioning in this context, replaces the Jacobian $\mathbf{J}$ by $\mathbf{J}\mathbf{S}$ for a positive diagonal $\mathbf{S}$, solves for a scaled increment, and recovers the true increment as $\xi = \mathbf{S}\eta$.

#### Derivation

The motivation is that condition number is not an intrinsic property of the estimation problem but partly an artefact of the units in which the variables happen to be expressed. A pose has translation components in metres and rotation components in radians, a velocity is in metres per second, a gyroscope bias is in radians per second and a landmark carries an inverse distance in reciprocal metres. The corresponding columns of the Jacobian have wildly different norms, and the resulting information matrix has a badly spread diagonal for reasons that have nothing to do with how well the problem is observed.

Scaling the columns transforms the information matrix by congruence.

$$((\mathbf{J}\mathbf{S}))^T((\mathbf{J}\mathbf{S})) = \mathbf{S}\,\mathbf{J}^T\mathbf{J}\,\mathbf{S}$$

The diagonal entries of the scaled matrix are $s_i^2\|\mathbf{j}_i\|^2$ for the $i$-th column $\mathbf{j}_i$. Choosing $s_i = 1/\|\mathbf{j}_i\|$ makes every diagonal entry exactly one, which is the best that a diagonal transformation can do and is within a factor of $\sqrt{n}$ of the optimal diagonal preconditioner by a theorem of van der Sluis. `basalt` uses a regularised form guarding against a column of zero norm, which occurs for a variable that no residual in the current problem touches.

$$s_i = \frac{1}{\epsilon + \|\mathbf{j}_i\|}$$

The implementation for landmark columns is at `landmark_block_abs_dynamic.hpp:374-385`, and the source notes the contrast with Ceres, which uses $1/(1 + \|\mathbf{j}_i\|)$ rather than an epsilon.

```cpp
// ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
// we use 1.0 / (eps + sqrt(SquaredColumnNorm))
Jl_col_scale =
    (options_->jacobi_scaling_eps +
     storage.block(0, lm_idx, num_rows - 3, 3).colwise().norm().array())
        .inverse();

storage.block(0, lm_idx, num_rows - 3, 3) *= Jl_col_scale.asDiagonal();
```

Note that the scaling is applied to the Jacobian itself rather than being carried as a separate matrix, which means the recovered increment is in scaled coordinates and must be unscaled before it is applied to the state.

#### Significance

Three implementation consequences follow, and each is a place where the code would be wrong if the corresponding step were omitted.

The scaling must be computed from all contributors jointly, because pose columns are shared between every landmark block, every inertial block and the prior. This is what `getJp_diag2` at `linearization_abs_qr.cpp:324-389` does, accumulating squared column norms from each. The source explicitly excludes damping from that accumulation, with a comment noting that the scaling should reflect the conditioning of the problem rather than of the regularisation, which is correct since including it would make the preconditioner depend on the damping parameter and therefore change within the inner loop.

The scaling of landmark columns must be undone on the recovered increment before it is applied, which happens at `landmark_block_abs_dynamic.hpp:331`, and must be undone after the model cost change has been computed rather than before, since that computation works in the scaled coordinates. The source carries a one line comment making the ordering explicit.

The scaling applied to the columns of the marginalisation prior must be recorded, because the prior is consumed again in later operations that must apply the same transformation. It is stored in the member `marg_scaling` and asserted to be set exactly once, at `linearization_abs_qr.cpp:460-467`.

Two ordering assertions guard the whole scheme, at `landmark_block_abs_dynamic.hpp:387-395`, requiring that pose column scaling be applied only after the elimination has run and only while no landmark damping is active. Both are necessary, the first because the reduced rows are what the global scaling vector describes and the second because scaling a matrix into which damping has been folded would scale the damping too.

### 2.15 Rank Deficiency, Gauge Freedom and Observability

#### Definition

A parameter direction is unobservable when moving the state along it leaves every measurement unchanged. The set of such directions is the nullspace of the Jacobian, and equivalently of the information matrix, and its existence is called gauge freedom.

#### Derivation

Visual-inertial odometry has exactly four unobservable directions, and identifying them is a matter of asking which transformations of the whole trajectory leave every measurement invariant.

A reprojection residual depends on the poses only through relative geometry, since it compares a landmark's projection in one frame against its observation in another. Applying a common rigid transformation to every pose and every landmark simultaneously therefore changes no reprojection residual whatever. That accounts for six directions, being three of translation and three of rotation.

The inertial measurements break two of them. The accelerometer measures specific force, which includes gravity, and gravity has a fixed direction in the world frame. Rotating the whole trajectory about a horizontal axis would tilt the gravity vector relative to the trajectory and change the predicted accelerometer readings, so roll and pitch are observable. Rotation about the gravity direction itself does not change the gravity vector, so yaw remains unobservable.

The surviving nullspace is therefore three dimensional in translation and one dimensional in rotation about gravity, giving four directions in total. Writing $\mathbf{g}$ for the gravity direction, the nullspace basis at a pose with rotation $\mathbf{R}$ and position $\mathbf{p}$ consists of the three global translations together with the global yaw generator.

$$\mathcal{N} = \operatorname{span}(\{\begin{bmatrix}\mathbf{e}_1\\ \mathbf{0}\end{bmatrix}, \begin{bmatrix}\mathbf{e}_2\\ \mathbf{0}\end{bmatrix}, \begin{bmatrix}\mathbf{e}_3\\ \mathbf{0}\end{bmatrix}, \begin{bmatrix}[\mathbf{g}]_\times\mathbf{p}\\ \mathbf{g}\end{bmatrix})\}$$

The consequence is that the information matrix is singular, with rank deficiency four, and the undamped normal equations have no unique solution. This is not a defect of the data or of the algorithm. It is a correct statement that the measurements do not determine where the origin is or which way is north, and any estimator that produced a unique answer would be asserting information it does not have.

A monocular system has a fifth unobservable direction, being the overall scale, which is why `doc/VIO.md` §2.4.1 treats scale ambiguity as a limitation of visual odometry. Inertial measurements fix the scale, since the accelerometer provides an absolute metric reference, which is one of the principal reasons for fusing them.

Three remedies are available in general. The gauge may be fixed by holding some variables constant, it may be fixed by adding a prior along the unobservable directions, or it may be left free and handled by a solver that tolerates rank deficiency. `basalt` uses the second at initialisation and the third thereafter.

#### Significance

Four consequences run through the system.

The initial prior of §3.2.2 anchors exactly the four unobservable directions and nothing else, placing a large weight on the three translation components and on yaw, at indices zero to two and index five, and deliberately leaving roll, pitch and velocity unweighted because they are observable. Weighting an observable direction would inject information the system does not possess and would bias the estimate.

The damping of §2.4 makes the system solvable at every subsequent iteration without any explicit gauge fixing, because adding a strictly positive quantity to the diagonal renders the matrix positive definite regardless of its rank. The clamp by `min_lambda` is what guarantees this.

The marginalisation prior must preserve the nullspace, which is the entire motivation for the First-Estimate Jacobians convention of §2.16. A prior that acquired rank along an unobservable direction would make the estimator believe it knows its own gauge, and the resulting over confidence is a well documented cause of inconsistency in sliding window estimators.

The system verifies this in operation. A parallel structure `nullspace_marg_data` is maintained alongside the true prior with the same ordering but no prior terms, at `sqrt_keypoint_vio.cpp:82-85`, and `checkMargNullspace` compares the two to confirm that no spurious information has appeared along the four directions.

The observability analysis for filtering estimators is due to Huang and colleagues [[17]](#bib-17), and the sliding window treatment closest to the present system is that of Leutenegger and colleagues [[19]](#bib-19).

### 2.16 First-Estimate Jacobians

#### Definition

First-Estimate Jacobians is the convention that once a variable has entered a marginalisation prior, every subsequent Jacobian involving that variable is evaluated at the estimate it held at the moment of marginalisation, called its linearisation point, while its residual continues to be evaluated at the current estimate.

#### Derivation

The problem the convention solves is a subtle inconsistency, and it is worth constructing carefully because the fix looks arbitrary otherwise.

The nullspace of §2.15 is a property of the Jacobians, and it is exact only when all Jacobians are evaluated at the same state. Write $\mathbf{J}(\mathbf{s})$ for the Jacobian evaluated at state $\mathbf{s}$ and $\mathcal{N}(\mathbf{s})$ for its nullspace, which as the expression in §2.15 shows depends on the state through the position $\mathbf{p}$.

Now consider what happens without the convention. At time $k$ a set of variables is marginalised, producing a prior whose information matrix $\mathbf{H}^\ast$ is built from Jacobians evaluated at $\mathbf{s}_k$, so its nullspace is $\mathcal{N}(\mathbf{s}_k)$. At time $k+1$ the surviving variables have moved to $\mathbf{s}_{k+1}$, and the new visual and inertial factors are linearised there, so their nullspace is $\mathcal{N}(\mathbf{s}_{k+1})$. The total information matrix is the sum of the two contributions, and the nullspace of a sum is the intersection of the nullspaces.

$$\mathcal{N}((\mathbf{H}_\text{total})) = \mathcal{N}(\mathbf{s}_k) \cap \mathcal{N}(\mathbf{s}_{k+1})$$

Since the two differ, the intersection is smaller than either, and generically it is trivial. The total information matrix therefore has full rank, and the estimator behaves as though it had determined its own gauge. The apparent information is entirely spurious, having been manufactured by the inconsistency of the linearisation points rather than supplied by any measurement, and the resulting covariance is too small. The estimator becomes over confident and, because it now resists corrections along directions it should treat as free, inconsistent.

The remedy is to force all Jacobians touching a marginalised variable to share one linearisation point, so that the two nullspaces coincide and their intersection is the correct four dimensional space. The cost is that the Jacobians are evaluated at a state that is no longer the best estimate, which introduces a linearisation error that grows as the estimate drifts from the frozen point. The trade is universally judged worthwhile, since the error is second order in the drift whereas the inconsistency is first order in its effect on the covariance.

Formally each affected variable carries two quantities, being a frozen linearisation point $\mathbf{s}_\text{lin}$ and an accumulated tangent offset $\delta$, with the current estimate recovered as $\mathbf{s}_\text{cur} = \mathbf{s}_\text{lin}\oplus\delta$. The rule is then stated in one line.

$$\mathbf{J} \text{ evaluated at } \mathbf{s}_\text{lin}, \qquad \mathbf{r} \text{ evaluated at } \mathbf{s}_\text{cur}$$

#### Significance

The convention is implemented uniformly and appears at four places, each of which is quoted in the sections that follow.

The state wrappers carry the mechanism. `PoseVelBiasStateWithLin::applyInc` at `imu_types.h:115-123` accumulates into `delta` rather than moving the linearisation point once the `linearized` flag is set, and `setLinTrue` at `imu_types.h:109-113` is the moment the flag is raised, asserting that the offset is zero at that instant.

The relative pose computation at `linearization_abs_qr.cpp:326-345` evaluates the Jacobians at `getPoseLin()` and recomputes only the value at `getPose()` when either endpoint is frozen.

The inertial block at `imu_block.hpp:44-54` evaluates the Jacobians at `getStateLin()` and recomputes only the residual at `getState()` under the same condition.

The prior itself is re-centred using the accumulated offset, which is gathered by `computeDelta` at `ba_base.cpp:315-332`. That routine asserts that every variable in the prior's ordering has been frozen, which is the runtime enforcement of the convention.

The mechanism by which $\delta$ enters the prior's residual is developed in §3.2.3 and is the single most important equation in the interaction between marginalisation and odometry.

### 2.17 Marginalisation as Gaussian Conditioning

#### Definition

Marginalising a variable out of a joint probability distribution means integrating the distribution over that variable, leaving a distribution over the remainder that accounts for everything the eliminated variable contributed.

$$p(\mathbf{x}_\alpha) = \int p(\mathbf{x}_\alpha, \mathbf{x}_\beta)\,\mathrm{d}\mathbf{x}_\beta$$

#### Derivation

For a Gaussian this integral has a closed form, and the point of this section is that the closed form is exactly the Schur complement of §2.10, which is why the algebraic operation carries a probabilistic name.

Let the joint distribution be Gaussian in information form, parameterised by an information matrix $\mathbf{H}$ and information vector $\mathbf{b}$ as in `doc/Marginalisation.md` §A.1.1, and partition to match.

$$-\log p(\mathbf{x}) = \frac{1}{2}\begin{bmatrix}\mathbf{x}_\alpha\\ \mathbf{x}_\beta\end{bmatrix}^T\begin{bmatrix}\mathbf{H}_{\alpha\alpha} & \mathbf{H}_{\alpha\beta}\\ \mathbf{H}_{\beta\alpha} & \mathbf{H}_{\beta\beta}\end{bmatrix}\begin{bmatrix}\mathbf{x}_\alpha\\ \mathbf{x}_\beta\end{bmatrix} - \begin{bmatrix}\mathbf{b}_\alpha\\ \mathbf{b}_\beta\end{bmatrix}^T\begin{bmatrix}\mathbf{x}_\alpha\\ \mathbf{x}_\beta\end{bmatrix} + \text{const}$$

Complete the square in $\mathbf{x}_\beta$. The terms involving it are $\frac{1}{2}\mathbf{x}_\beta^T\mathbf{H}_{\beta\beta}\mathbf{x}_\beta + \mathbf{x}_\beta^T(\mathbf{H}_{\beta\alpha}\mathbf{x}_\alpha - \mathbf{b}_\beta)$, which is minimised at the value below and which can be written as a perfect square plus a remainder independent of $\mathbf{x}_\beta$.

$$\mathbf{x}_\beta^\ast = \mathbf{H}_{\beta\beta}^{-1}((\mathbf{b}_\beta - \mathbf{H}_{\beta\alpha}\mathbf{x}_\alpha))$$

The Gaussian integral over the perfect square contributes a constant independent of $\mathbf{x}_\alpha$, since the integral of a Gaussian is determined by its covariance alone. What survives is the remainder, and collecting terms gives the following.

$$-\log p(\mathbf{x}_\alpha) = \frac{1}{2}\mathbf{x}_\alpha^T((\mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{H}_{\beta\alpha}))\mathbf{x}_\alpha - ((\mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{b}_\beta))^T\mathbf{x}_\alpha + \text{const}$$

The information matrix of the marginal is the Schur complement and the information vector is the corresponding reduced vector. The algebraic elimination and the probabilistic integration are the same operation.

Two properties of this result are worth stating because they are frequently misunderstood.

The operation is exact for a Gaussian and involves no approximation whatever. The approximation in a sliding window estimator lies entirely in the linearisation that produced the Gaussian in the first place, not in the marginalisation of it. This is why the marginalisation is described as information lossless. The fixed lag smoother built on this operation, in which a bounded window of recent states is optimised while older ones are summarised into a prior, is due to Sibley and colleagues [[18]](#bib-18) and is the architecture this estimator implements.

The operation destroys sparsity. Before marginalisation, two surviving variables are connected only if some measurement involved both. After marginalisation, every pair of surviving variables that were both connected to the eliminated variable becomes directly connected, because the term $\mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{H}_{\beta\alpha}$ is dense over the Markov blanket. This fill in is why the prior is a dense matrix, why the window must be kept small, and why nonlinear factor recovery [[3]](#bib-3) [[4]](#bib-4) [[5]](#bib-5) exists as a means of re-sparsifying the result for the mapping layer.

#### Significance

This reading supplies the vocabulary the rest of the document uses. The set of surviving variables directly connected to an eliminated one is its Markov blanket, and it is exactly the set over which the new prior is dense. The prior is a probability distribution over the surviving variables, which is why it can be converted into a synthetic residual, as `doc/Marginalisation.md` Appendix A derives, and why that residual behaves like any other measurement in the objective.

It also explains why the keyframe selection heuristics of `doc/Marginalisation.md` §3.1 matter. Choosing which keyframe to eliminate is choosing where to introduce fill in, and eliminating a keyframe with a large Markov blanket produces a denser prior than eliminating a redundant one. The distance based score exists to prefer the eliminations that damage sparsity least.
---

## 3. Linearisation

### 3.1 Absolute QR Linearisation

Absolute QR linearisation is the strategy selected by `LinearizationType::ABS_QR`, the default value set at `src/utils/vio_config.cpp:58`. The name encodes two independent choices. Absolute means that the unknowns are the absolute poses of the frames in the world frame, rather than the relative poses between frame pairs that the alternative `REL_SC` strategy uses. QR means that landmarks are eliminated by orthogonal projection onto the left nullspace of the landmark Jacobian, as derived in §2.7, rather than by the Schur complement that `ABS_SC` and `REL_SC` use. The free function `isLinearizationSqrt` at `src/linearization/linearization_base.cpp:45-56` returns true for `ABS_QR` alone, and that single boolean governs which code paths the estimator takes at several later points.

This subsection develops the strategy in the order in which the implementation performs it, since the mathematics and the data structures are tightly coupled and neither is intelligible alone.

#### 3.1.1 The Absolute Parameterisation and the Ordering of Variables

The unknowns of the problem divide into two groups whose treatment differs throughout. The frame variables comprise the six degree of freedom poses of older keyframes, held in `frame_poses`, and the fifteen degree of freedom navigation states of recent frames, held in `frame_states`. The landmark variables comprise three parameters per landmark, held in the landmark database. The distinction matters because the frame variables survive into the solved system while the landmarks are eliminated before it.

The layout of the frame variables is fixed by an `AbsOrderMap`, defined at `include/basalt/utils/imu_types.h:293-305`.

```cpp
struct AbsOrderMap {
  std::map<int64_t, std::pair<int, int>> abs_order_map;
  size_t items = 0;
  size_t total_size = 0;
};
```

Each entry maps a frame timestamp to a pair holding the column offset at which that frame's block begins and the width of that block, which is `POSE_SIZE`, equal to six, for a pose only keyframe and `POSE_VEL_BIAS_SIZE`, equal to fifteen, for a full navigation state. The member `total_size` accumulates to the total column count of the linear system and `items` counts the blocks. Because the underlying container is a `std::map` keyed by timestamp, iteration visits frames from oldest to newest, and every construction site in the codebase exploits this by iterating `frame_poses` first and `frame_states` second.

The resulting layout is therefore all pose only keyframes in ascending time, followed by all full navigation states in ascending time. This is not an arbitrary convention. It is the invariant that permits the marginalisation prior to be applied without any index remapping, because the variables the prior constrains are always exactly the oldest ones and therefore always occupy a contiguous prefix of the columns. §3.2.3 develops the consequence in full, and the assertions that enforce it are quoted there.

Within a fifteen wide navigation state block the tangent ordering is fixed by `PoseVelBiasState::applyInc` in `thirdparty/basalt-headers/include/basalt/imu/imu_types.h:219-224`, whose own doxygen comment states it.

```cpp
/// @param[in] inc 15x1 increment vector [trans, rot, vel, bias_gyro,
/// bias_accel]
void applyInc(const VecN& inc) {
  PoseVelState<Scalar>::applyInc(inc.template head<9>());
  bias_gyro += inc.template segment<3>(9);
  bias_accel += inc.template segment<3>(12);
}
```

The indices are therefore translation at zero to two, rotation at three to five, velocity at six to eight, gyroscope bias at nine to eleven and accelerometer bias at twelve to fourteen. This ordering is needed to read the initial prior of §3.2.2 correctly, and misreading it is the origin of a defect recorded there.

#### 3.1.2 Input Data to the Linearisation

A `LinearizationAbsQR` object is constructed with seven arguments beyond the scalar type, and understanding what each supplies is the clearest route into the class. The factory signature is declared at `include/basalt/linearization/linearization_base.hpp:51-58`.

```cpp
static std::unique_ptr<LinearizationBase> create(
    BundleAdjustmentBase<Scalar>* estimator, const AbsOrderMap& aom,
    const Options& options,
    const MargLinData<Scalar>* marg_lin_data = nullptr,
    const ImuLinData<Scalar>* imu_lin_data = nullptr,
    const std::set<FrameId>* used_frames = nullptr,
    const std::unordered_set<KeypointId>* lost_landmarks = nullptr,
    int64_t last_state_to_marg = std::numeric_limits<int64_t>::max());
```

The estimator pointer supplies everything the linearisation reads but does not own, namely the landmark database `lmdb`, the two state containers `frame_poses` and `frame_states`, the calibration, the noise and robustifier settings, and a set of helper methods for the marginalisation prior. It is stored as a pointer to const at `linearization_abs_qr.hpp:116`, so the linearisation cannot mutate the estimator. The landmark database, the pose container and the calibration are bound as references rather than copied, which makes construction cheap but means the estimator must not be mutated concurrently.

The order map supplies the column layout of §3.1.1 and is likewise stored by reference.

The options struct carries two fields only, being the landmark block options that are forwarded verbatim into every landmark block, and the enum that the factory switches on. The constructor asserts at `linearization_abs_qr.cpp:69-74` that the Huber threshold and the observation standard deviation in those options match the estimator's own, which guards against a caller constructing options that have drifted from the configuration.

The marginalisation pointer supplies the prior inherited from the previous marginalisation, or null when none exists. The inertial pointer supplies the preintegrated measurements together with gravity and the two bias random walk weights, or null for a purely visual problem. This second pointer is the only thing that distinguishes visual odometry from visual-inertial odometry in this class. There is no separate template instantiation and no separate subclass, since every inertial code path is guarded by a null check on `imu_lin_data`, and both scalar instantiations at `linearization_base.cpp:96-117` fix `POSE_SIZE` to six regardless.

The two filter arguments restrict which landmarks are linearised. When neither is supplied every landmark in the database is included, which is what the odometry path wants. When either is supplied a landmark is included only if its host keyframe lies in `used_frames` or the landmark itself lies in `lost_landmarks`, which is what the marginalisation path wants. The selection is at `linearization_abs_qr.cpp:116-127`.

```cpp
for (const auto& [k, v] : lmdb_.getLandmarks()) {
  if (used_frames || lost_landmarks) {
    if (used_frames && used_frames->count(v.host_kf_id.frame_id)) {
      landmark_ids.emplace_back(k);
    } else if (lost_landmarks && lost_landmarks->count(k)) {
      landmark_ids.emplace_back(k);
    }
  } else {
    landmark_ids.emplace_back(k);
  }
}
```

The final argument, `last_state_to_marg`, is accepted and immediately discarded by `UNUSED(last_state_to_marg)` at `linearization_abs_qr.cpp:67`. It plays no part in this class and is consumed instead by the estimator itself when truncating the order map. It is worth recording as a piece of dead interface, since a reader may otherwise search for its effect at length.

#### 3.1.3 The Relative Pose Cache

The reprojection residual of a landmark hosted in frame $h$ and observed in frame $t$ depends on the two absolute poses only through their composition into a relative pose. Since many landmarks share the same host and target pair, computing that composition and its Jacobians once per pair rather than once per observation is a substantial saving, and it is the first optimisation the class applies.

The constructor walks the observation structure of the landmark database, which is keyed by host and then by target, and creates one default constructed `RelPoseLin` entry per host and target pair. The values are filled at the start of every call to `linearizeProblem`, at `linearization_abs_qr.cpp:192-228`.

```cpp
Sophus::SE3<Scalar> T_t_h_sophus =
    computeRelPose(state_h.getPoseLin(), calib.T_i_c[tcid_h.cam_id],
                   state_t.getPoseLin(), calib.T_i_c[tcid_t.cam_id],
                   &rpl.d_rel_d_h, &rpl.d_rel_d_t);

if (state_h.isLinearized() || state_t.isLinearized()) {
  T_t_h_sophus =
      computeRelPose(state_h.getPose(), calib.T_i_c[tcid_h.cam_id],
                     state_t.getPose(), calib.T_i_c[tcid_t.cam_id]);
}

rpl.T_t_h = T_t_h_sophus.matrix();
```

The structure of this fragment is the First-Estimate Jacobians convention of §2.11 in its purest form. The first call evaluates both the relative pose and its two Jacobians at the frozen linearisation points returned by `getPoseLin`. The second call, taken only when either endpoint has been frozen, recomputes the relative pose value alone at the current estimates returned by `getPose`, and deliberately does not recompute the Jacobians. The residual therefore reflects where the states actually are while the Jacobians reflect where they were when first marginalised, which is exactly what preserves the nullspace.

The relative pose composition and its Jacobians are computed by `computeRelPose` in `include/basalt/utils/ba_utils.h`. Writing $\mathbf{T}_{th}$ for the relative pose from the host camera to the target camera, the composition is as follows, where $\mathbf{T}_{ic}$ denotes the camera to body extrinsic of the relevant camera.

$$\mathbf{T}_{th} = \mathbf{T}_{ic,t}^{-1}\,\mathbf{T}_{wi,t}^{-1}\,\mathbf{T}_{wi,h}\,\mathbf{T}_{ic,h}$$

The two Jacobians with respect to the tangent perturbations of the host and target body poses are built from the adjoint of the intermediate composition together with a block diagonal rotation, and carry opposite signs because the host enters the product directly while the target enters through an inverse.

```cpp
if (d_rel_d_h) {
  Sophus::Matrix3<Scalar> R = T_w_i_h.so3().inverse().matrix();
  Sophus::Matrix6<Scalar> RR;
  RR.setZero();
  RR.template topLeftCorner<3, 3>() = R;
  RR.template bottomRightCorner<3, 3>() = R;
  *d_rel_d_h = tmp.Adj() * RR;
}

if (d_rel_d_t) {
  Sophus::Matrix3<Scalar> R = T_w_i_t.so3().inverse().matrix();
  Sophus::Matrix6<Scalar> RR;
  RR.setZero();
  RR.template topLeftCorner<3, 3>() = R;
  RR.template bottomRightCorner<3, 3>() = R;
  *d_rel_d_t = -tmp2.Adj() * RR;
}
```

One asymmetry in the constructor deserves note. The relative pose cache is built for every host and target pair present in the landmark database, without applying the `used_frames` filter, because the filtering line is commented out at `linearization_abs_qr.cpp:78`. Only the landmark selection is filtered. During marginalisation, when the filter is active and only a few landmarks are linearised, the cache is therefore larger than strictly necessary. The waste is bounded by the number of frame pairs rather than the number of landmarks and is modest, but it is a genuine inefficiency rather than a design intent.

#### 3.1.4 The Reprojection Residual and its Jacobians as Computed

`doc/VIO.md` §2.2.4 and §2.2.5 derive the reprojection residual and its Jacobians. What follows is the computational form actually evaluated, given so that the code can be read against the mathematics. The function is `linearizePoint` in `include/basalt/utils/ba_utils.h:83-144`.

The landmark is parameterised in the host frame by a two parameter stereographic direction and a scalar inverse distance, assembled into a homogeneous four vector.

$$\tilde{\mathbf{p}}_h = \begin{bmatrix}\pi_{st}^{-1}(\boldsymbol{\theta}_j)\\ \rho_j\end{bmatrix} \in \mathbb{R}^4$$

This is transformed into the target frame by the relative pose and projected through the target camera model, and the observation is subtracted.

$$\tilde{\mathbf{p}}_t = \mathbf{T}_{th}\,\tilde{\mathbf{p}}_h, \qquad \mathbf{r}_{ij} = \pi(\tilde{\mathbf{p}}_t) - \mathbf{z}_{ij}$$

The homogeneous parameterisation is what keeps the residual well defined as $\rho_j$ approaches zero, which is to say for landmarks at great distance, since no division by the depth is required. This is the standard device of projective geometry, treated in Hartley and Zisserman [[20]](#bib-20), applied here to the inverse distance parameterisation rather than to image coordinates.

The projection itself is dispatched at runtime on the camera model, through a `std::visit` over the calibration variant, so the same block serves a pinhole, a Kannala-Brandt or a double sphere camera [[21]](#bib-21) without any change to the surrounding algebra. Only the two by four Jacobian $\mathbf{J}_\pi$ differs between them.

The Jacobian with respect to the relative pose tangent is formed by the chain rule through the projection Jacobian $\mathbf{J}_\pi$, which is two by four.

$$\frac{\partial \tilde{\mathbf{p}}_t}{\partial \xi} = \begin{bmatrix}\rho_j\mathbf{I}_3 & -[\mathbf{p}_t]_\times\\ \mathbf{0}^T & \mathbf{0}^T\end{bmatrix} \in \mathbb{R}^{4\times 6}, \qquad \frac{\partial \mathbf{r}_{ij}}{\partial \xi} = \mathbf{J}_\pi \frac{\partial \tilde{\mathbf{p}}_t}{\partial \xi}$$

The Jacobian with respect to the three landmark parameters uses the stereographic unprojection Jacobian $\mathbf{J}_{up}$ for the two direction components, and for the inverse distance component reduces to the translation column of the relative pose, since the homogeneous product gives $\partial\tilde{\mathbf{p}}_t/\partial\rho_j = \mathbf{t}_{th}$.

$$\frac{\partial \tilde{\mathbf{p}}_t}{\partial \mathbf{l}_j} = \begin{bmatrix}\mathbf{R}_{th}\mathbf{J}_{up} & \mathbf{t}_{th}\end{bmatrix} \in \mathbb{R}^{4\times 3}, \qquad \frac{\partial \mathbf{r}_{ij}}{\partial \mathbf{l}_j} = \mathbf{J}_\pi \frac{\partial \tilde{\mathbf{p}}_t}{\partial \mathbf{l}_j}$$

These two Jacobians are with respect to the relative pose, which is not an unknown of the problem. They are converted to the absolute unknowns by a further chain rule through the cached relative pose Jacobians of §3.1.3, giving the two six column blocks that are actually written into storage.

$$\frac{\partial \mathbf{r}_{ij}}{\partial \xi_h} = \frac{\partial \mathbf{r}_{ij}}{\partial \xi}\,\frac{\partial \xi}{\partial \xi_h}, \qquad \frac{\partial \mathbf{r}_{ij}}{\partial \xi_t} = \frac{\partial \mathbf{r}_{ij}}{\partial \xi}\,\frac{\partial \xi}{\partial \xi_t}$$

A landmark is marked as a numerical failure when either Jacobian contains a non-finite entry, or when the projection itself reports invalidity, which occurs when the point falls behind the camera. The check is at `landmark_block_abs_dynamic.hpp:163-165` and the resulting state is propagated up through `linearizeProblem` as a boolean that tells the caller the whole linearisation point is untrustworthy.

#### 3.1.5 The Landmark Block and its Storage Matrix

The landmark block is the central data structure of the whole design, and its single dense storage matrix is what makes the elimination of §3.1.6 a three line operation. One block is built per selected landmark, in parallel, and holds every quantity relating to that landmark and nothing else.

The declaration is at `landmark_block_abs_dynamic.hpp:558-559`.

```cpp
// Dense storage for pose Jacobians, padding, landmark Jacobians and
// residuals [J_p | pad | J_l | res]
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    storage;
```

The sizing is computed in `allocateLandmark` at `landmark_block_abs_dynamic.hpp:84-102`.

```cpp
// number of pose-jacobian columns is determined by oam
padding_idx = aom_->total_size;

num_rows = pose_lin_vec.size() * 2 + 3;  // residuals and lm damping

size_t pad = padding_idx % 4;
if (pad != 0) {
  padding_size = 4 - pad;
}

lm_idx = padding_idx + padding_size;
res_idx = lm_idx + 3;
num_cols = res_idx + 1;

// number of columns should now be multiple of 4 for good memory alignment
// TODO: test extending this to 8 --> 32byte alignment for float?
BASALT_ASSERT(num_cols % 4 == 0);

storage.resize(num_rows, num_cols);
```

Writing $n_\text{obs}$ for the number of observations of this landmark and $N$ for `aom.total_size`, the shape is as follows.

| Region | Column range | Width | Contents |
|---|---|---|---|
| $\mathbf{J}_p$ | $[0, N)$ | $N$ | pose Jacobian over the entire active state |
| padding | $[N, N + p)$ | $p \in \{0,1,2,3\}$ | never written, alignment only |
| $\mathbf{J}_l$ | $[N+p, N+p+3)$ | 3 | landmark Jacobian |
| $\mathbf{r}$ | $N+p+3$ | 1 | residual |

| Region | Row range | Height | Contents |
|---|---|---|---|
| observations | $[0, 2n_\text{obs})$ | $2n_\text{obs}$ | two rows per observation |
| damping | $[2n_\text{obs}, 2n_\text{obs}+3)$ | 3 | reserved for landmark damping |

Four properties of this layout deserve comment, because each is a deliberate choice with consequences.

The pose Jacobian spans the entire active state, not merely the frames this landmark observes. The comment at `landmark_block_abs_dynamic.hpp:85` states it directly, and a `TODO` at lines 52 to 55 acknowledges the resulting waste and proposes a reduced order map as a remedy. The block is therefore structurally sparse but densely stored, with the columns of unobserved frames left at zero. The cost is affordable only because the sliding window bounds $N$ to a few hundred scalars. The benefit is that a single orthogonal transformation can be applied across a whole row with no index bookkeeping whatever, which is what reduces the elimination to the loop of §3.1.6. Sparsity is not thereby discarded, since the block separately maintains `res_idx_by_abs_pose_`, a map from frame identifier to the set of observation indices that touch it, and the operations that can exploit sparsity, such as `addJp_diag2`, use it.

The padding rounds the pose Jacobian width up to a multiple of four so that the landmark and residual columns, which number four together, keep the total a multiple of four. The purpose is memory alignment for vectorised row operations, and the `TODO` asks whether eight would be better for single precision.

The two numbers three in the layout are unrelated in origin though equal in value. Three columns because the landmark carries three parameters. Three rows because Levenberg-Marquardt damping of those three parameters requires three rows. That they coincide is convenient, since the row split of the elimination falls at row three either way, but a reader should not infer a single cause.

Finally, the residual is stored as a column of the same matrix rather than separately. This is what allows the orthogonal transformation to be applied to the residual in lock step with the Jacobian without a second call, since a transformation applied across all columns transforms the residual too.

The block is filled by `linearizeLandmark` at `landmark_block_abs_dynamic.hpp:114-201`, whose essential lines are the following.

```cpp
const Scalar res_squared = res.squaredNorm();
const auto [weighted_error, weight] = compute_error_weight(res_squared);
const Scalar sqrt_weight = std::sqrt(weight) / options_->obs_std_dev;

error_sum += weighted_error / (options_->obs_std_dev * options_->obs_std_dev);

storage.template block<2, 3>(obs_idx, lm_idx) = sqrt_weight * d_res_d_p;
storage.template block<2, 1>(obs_idx, res_idx) = sqrt_weight * res;

d_res_d_xi *= sqrt_weight;
storage.template block<2, 6>(obs_idx, abs_h_idx) +=
    d_res_d_xi * pose_lin_vec[i]->d_rel_d_h;
storage.template block<2, 6>(obs_idx, abs_t_idx) +=
    d_res_d_xi * pose_lin_vec[i]->d_rel_d_t;
```

The whitening of §2.2 is the scalar `sqrt_weight`, applied to the landmark Jacobian, the residual and the pose Jacobian alike so that the stored rows are already in the units in which an orthogonal transformation is legitimate. The accumulation into the pose columns uses `+=` rather than `=` because a single frame may act as host for one observation and target for another, in which case two contributions land in the same six columns and must sum.

#### 3.1.6 The Method performQR and the Elimination of the Landmark

With the storage matrix filled, the elimination of the landmark is the application of §2.7 to a single block. The public entry point at `landmark_block_abs_dynamic.hpp:203-216` selects between two implementations and advances the block state.

```cpp
virtual inline void performQR() override {
  BASALT_ASSERT(state == State::Linearized);

  // Since we use dense matrices Householder QR might be better:
  // https://mathoverflow.net/questions/227543/why-householder-reflection-is-better-than-givens-rotation-in-dense-linear-algebr

  if (options_->use_householder) {
    performQRHouseholder();
  } else {
    performQRGivens();
  }

  state = State::Marginalized;
}
```

The default implementation, at `landmark_block_abs_dynamic.hpp:456-472`, is three Householder reflections.

```cpp
inline void performQRHouseholder() {
  VecX tempVector1(num_cols);
  VecX tempVector2(num_rows - 3);

  for (size_t k = 0; k < 3; ++k) {
    size_t remainingRows = num_rows - k - 3;

    Scalar beta;
    Scalar tau;
    storage.col(lm_idx + k)
        .segment(k, remainingRows)
        .makeHouseholder(tempVector2, tau, beta);

    storage.block(k, 0, remainingRows, num_cols)
        .applyHouseholderOnTheLeft(tempVector2, tau, tempVector1.data());
  }
}
```

The mechanism is worth stating precisely because a single line carries the whole idea. Each iteration computes a reflection from one column of the landmark Jacobian, specifically column $k$ over the rows from $k$ down to the last observation row, chosen so as to annihilate every entry of that column below the diagonal. The reflection is then applied not to that column, nor to the landmark Jacobian, but to `storage.block(k, 0, remainingRows, num_cols)`, which is to say to every column of the matrix over that row range. The pose Jacobian and the residual are therefore transformed by exactly the same orthogonal matrix as the landmark Jacobian, in lock step, which is the requirement that makes the transformation an identity on the objective rather than a change to it. The orthogonal matrix $\mathbf{Q}$ is never formed, since only its action is needed.

After three iterations the storage matrix holds the block structure of §2.7, and the row split falls at row three.

$$\text{rows } [0,3) \;=\; \begin{bmatrix}\mathbf{Q}_1^T\mathbf{J}_p & \mathbf{R} & \mathbf{Q}_1^T\mathbf{r}\end{bmatrix}, \qquad \text{rows } [3, 2n_\text{obs}) \;=\; \begin{bmatrix}\mathbf{Q}_2^T\mathbf{J}_p & \mathbf{0} & \mathbf{Q}_2^T\mathbf{r}\end{bmatrix}$$

The first three rows retain all dependence on the landmark, in the three by three upper triangular factor $\mathbf{R}$ sitting in the landmark columns, and are kept for the back substitution of §3.1.10. The remaining rows are free of the landmark entirely and are the landmark's contribution to the reduced system over the frame variables. No inverse has been computed, no information has been discarded, and the landmark has been removed from the system that will be solved.

The source carries the clearest statement of the mathematics in a comment inside `backSubstitute`, at `landmark_block_abs_dynamic.hpp:280-290`, reproduced here since it is the authoritative in-code explanation.

```
// Here we have J = [Jp, Jl] under the orthogonal projection Q = [Q1, Q2],
// i.e. the linearized system (model cost) is
//
//    L(inc) = 0.5 || J inc + r ||^2 = 0.5 || Q^T J inc + Q^T r ||^2
//
//             | Q1^T |            | Q1^T Jp   Q1^T Jl |
//    Q^T J =  |      | [Jp, Jl] = |                   |
//             | Q2^T |            | Q2^T Jp      0    |.
//
// Note that Q2 is the nullspace of Jl, and Q1^T Jl == R.
```

The alternative implementation, `performQRGivens` at `landmark_block_abs_dynamic.hpp:444-454`, achieves the same reduction by sweeping plane rotations up each landmark column, citing Algorithm 5.2.4 of Golub and Van Loan by page number. It is not the default and exists chiefly as a reference implementation.

One accounting subtlety must be recorded, because it is easy to misread. The method `numQ2rows` returns `num_rows - 3`, which equals $2n_\text{obs}$, and the export method reads rows $[3, \text{num\_rows})$, which is also $2n_\text{obs}$ rows. The exported range therefore begins at the first genuine reduced row and runs past the last observation row into the three damping rows. When landmark damping is inactive those three rows are identically zero and contribute nothing, so the effective count of informative reduced rows is $2n_\text{obs} - 3$, which is the expected value since three degrees of freedom were consumed by the landmark. The three surplus rows are carried so that the exported row count is independent of whether damping is active, which keeps the global row offsets of §3.1.9 stable across the inner loop of the solver.

#### 3.1.7 Landmark Damping and its Exact Reversal

Damping a landmark means adding $\lambda$ to the diagonal of its three by three information block, which by §2.9 means appending three rows carrying $\sqrt{\lambda}$ to its Jacobian. Those rows must then be folded into the already triangular factor, and, since the solver varies $\lambda$ across the inner loop, the folding must be reversible without repeating the decomposition. The implementation at `landmark_block_abs_dynamic.hpp:219-259` achieves both.

```cpp
if (hasLandmarkDamping()) {
  BASALT_ASSERT(damping_rotations.size() == 6);

  // undo dampening
  for (int n = 2; n >= 0; n--) {
    for (int m = n; m >= 0; m--) {
      storage.applyOnTheLeft(num_rows - 3 + n - m, n,
                             damping_rotations.back().adjoint());
      damping_rotations.pop_back();
    }
  }
}
```

```cpp
storage.template block<3, 3>(num_rows - 3, lm_idx)
    .diagonal()
    .setConstant(sqrt(lambda));

// apply dampening and remember rotations to undo
for (int n = 0; n < 3; n++) {
  for (int m = 0; m <= n; m++) {
    damping_rotations.emplace_back();
    damping_rotations.back().makeGivens(
        storage(n, lm_idx + n),
        storage(num_rows - 3 + n - m, lm_idx + n));
    storage.applyOnTheLeft(num_rows - 3 + n - m, n,
                           damping_rotations.back());
  }
}
```

The nested loop generates exactly six rotations, since three plus two plus one is six, and each zeroes one damping row entry against the corresponding diagonal entry of the triangular factor. Because each rotation is applied to the full row pair across all columns, the reduced pose Jacobian rows and the residual absorb the damping consistently. Storing the six rotations allows the undo path to replay them in reverse order using the adjoint of each, restoring the undamped factor exactly rather than approximately. This is the reason Givens rotations rather than Householder reflections are used here, since only six entries need removing and each must be individually invertible.

The method `backSubstitute` calls `setLandmarkDamping(0)` before computing the model cost change, at `landmark_block_abs_dynamic.hpp:292`, so that the predicted decrease is measured against the undamped model. This is correct, since the damping is a device for controlling the step and not part of the objective whose decrease is being predicted.

#### 3.1.8 Column Scaling

The scaling of §2.10 is applied in two places with different scope. Landmark columns are scaled by `scaleJl_cols` at `landmark_block_abs_dynamic.hpp:374-385`, which computes a three vector of factors from the column norms of the landmark Jacobian and stores it in the member `Jl_col_scale` so that the recovered increment can be unscaled later.

```cpp
// ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
// we use 1.0 / (eps + sqrt(SquaredColumnNorm))
Jl_col_scale =
    (options_->jacobi_scaling_eps +
     storage.block(0, lm_idx, num_rows - 3, 3).colwise().norm().array())
        .inverse();

storage.block(0, lm_idx, num_rows - 3, 3) *= Jl_col_scale.asDiagonal();
```

Pose columns are scaled by `scaleJp_cols`, which requires a global scaling vector because those columns are shared across all landmark blocks, all inertial blocks and the prior. That vector is derived from `getJp_diag2` at `linearization_abs_qr.cpp:324-389`, which accumulates the squared column norms of every contributor. Two assertions guard the ordering of operations, namely that pose scaling is applied only after the elimination has run and only while no landmark damping is active, at `landmark_block_abs_dynamic.hpp:387-395`.

#### 3.1.9 Assembly of the Global System

Two exits from the class produce a global system, and they differ in form rather than in content.

The square root exit, `get_dense_Q2Jp_Q2r` at `linearization_abs_qr.cpp:483-534`, stacks rows. Four contributors are laid out in a fixed order, each at an offset computed arithmetically from the sizes of its predecessors.

```cpp
size_t total_size = num_rows_Q2r;
size_t poses_size = aom.total_size;
size_t lm_start_idx = 0;
size_t imu_start_idx = total_size;
if (imu_lin_data) {
  total_size += imu_lin_data->imu_meas.size() * POSE_VEL_BIAS_SIZE;
}
size_t damping_start_idx = total_size;
if (hasPoseDamping()) {
  total_size += poses_size;
}
size_t marg_start_idx = total_size;
if (marg_lin_data) total_size += marg_lin_data->H.rows();

Q2Jp.setZero(total_size, poses_size);
Q2r.setZero(total_size);
```

The resulting matrix has $N$ columns, one per scalar unknown among the frame variables, and a row count that is the sum of the reduced landmark rows, fifteen rows per inertial factor, $N$ damping rows when pose damping is active, and the row count of the prior. The block row order is landmarks, then inertial factors, then damping, then prior. Absent contributors occupy no rows at all rather than zero rows, since the offsets are computed conditionally.

The normal equation exit, `get_dense_H_b` at `linearization_abs_qr.cpp:536-591`, accumulates instead of stacking. Each contributor adds into a pre-zeroed $N \times N$ matrix and $N$ vector using its own knowledge of which columns it touches. For a landmark block the accumulation is at `landmark_block_abs_dynamic.hpp:519-525` and is precisely the normal equations of the reduced rows.

```cpp
void add_dense_H_b(MatX& H, VecX& b) const override {
  const auto r = storage.col(res_idx).tail(num_rows - 3);
  const auto J = storage.block(3, 0, num_rows - 3, padding_idx);

  H.noalias() += J.transpose() * J;
  b.noalias() += J.transpose() * r;
}
```

This makes the relationship between the two exits explicit. Writing $\mathbf{J} = \mathbf{Q}_2^T\mathbf{J}_p$ and $\mathbf{r} = \mathbf{Q}_2^T\mathbf{r}$ for the reduced quantities, the second exit returns $\mathbf{H} = \mathbf{J}^T\mathbf{J}$ and $\mathbf{b} = \mathbf{J}^T\mathbf{r}$ summed over contributors, which is the normal equations of exactly the system the first exit returns in factored form. Note that $\mathbf{b}$ here carries a positive sign rather than the negative sign of the convention in §1.1, so the caller must negate the solved increment, which it does at `sqrt_keypoint_vio.cpp:1388`.

The choice between the exits is made by the caller and is discussed in §3.2.4 and §3.3.

#### 3.1.10 Back Substitution and the Model Cost Change

Once the reduced system has been solved for the frame increment, each landmark's own increment is recovered from the three rows that were set aside. The implementation at `landmark_block_abs_dynamic.hpp:262-335` performs a three by three triangular solve.

```cpp
const auto Q1Jl = storage.template block<3, 3>(0, lm_idx)
                      .template triangularView<Eigen::Upper>();
const auto Q1Jr = storage.col(res_idx).template head<3>();
const auto Q1Jp = storage.topLeftCorner(3, padding_idx);

Vec3 inc = -Q1Jl.solve(Q1Jr + Q1Jp * pose_inc);
```

This is the equation for $\xi_l^\ast$ derived in §2.7, realised without forming an inverse, since $\mathbf{R}$ is triangular and a triangular solve suffices. The same routine accumulates that landmark's share of the predicted cost decrease, using the identity of §2.1 and exploiting the fact that the reduced rows contain no landmark term.

```cpp
// compute "Q^T J incp"
VecX QJinc = storage.topLeftCorner(num_rows - 3, padding_idx) * pose_inc;

// add "Q1^T Jl incl" to the first 3 rows
QJinc.template head<3>() += Q1Jl * inc;

auto Qr = storage.col(res_idx).head(num_rows - 3);
l_diff -= QJinc.transpose() * (Scalar(0.5) * QJinc + Qr);
```

The increment is unscaled by the landmark column scaling and applied to the state, with the inverse distance clamped below at zero so that a landmark cannot be pushed behind its host camera.

```cpp
// Note: scale only after computing model cost change
inc.array() *= Jl_col_scale.array();

lm_ptr->direction += inc.template head<2>();
lm_ptr->inv_dist = std::max(Scalar(0), lm_ptr->inv_dist + inc[2]);
```

#### 3.1.11 Naming Conventions

The identifiers in this part of the codebase are terse and are opaque without the derivation of §2.7. The following table is the key, and it is worth reading before any of the source.

| Identifier | Mathematics | Meaning |
|---|---|---|
| `Jp` | $\mathbf{J}_p$ | Jacobian with respect to the frame variables, the poses and, where present, velocity and biases |
| `Jl` | $\mathbf{J}_l$ | Jacobian with respect to the three landmark parameters |
| `r`, `res` | $\mathbf{r}$ | residual, already whitened by $\sqrt{w_H}/\sigma_\text{obs}$ |
| `Q1`, `Q2` | $\mathbf{Q}_1$, $\mathbf{Q}_2$ | the two column blocks of the orthogonal factor of $\mathbf{J}_l$, the second spanning its left nullspace |
| `Q2Jp` | $\mathbf{Q}_2^T\mathbf{J}_p$ | the reduced pose Jacobian, free of the landmark |
| `Q2r` | $\mathbf{Q}_2^T\mathbf{r}$ | the reduced residual |
| `Q1Jl` | $\mathbf{Q}_1^T\mathbf{J}_l = \mathbf{R}$ | the triangular factor, held in the landmark columns of the first three rows |
| `Q1Jp`, `Q1Jr` | $\mathbf{Q}_1^T\mathbf{J}_p$, $\mathbf{Q}_1^T\mathbf{r}$ | the retained rows used for back substitution |
| `H`, `b` | $\mathbf{H}$, $\mathbf{b}$ | normal equation pair, with $\mathbf{b}$ positive signed in this code |
| `marg_lin_data->H` | $\mathbf{J}_\text{marg}$ or $\mathbf{H}^\ast$ | square root factor when `is_sqrt`, information matrix otherwise |
| `marg_lin_data->b` | $\mathbf{r}_\text{marg}$ or $\mathbf{b}^\ast$ | residual when `is_sqrt`, information vector otherwise |
| `aom` | ordering | the `AbsOrderMap` giving column offsets |
| `padding_idx` | $N$ | width of the pose Jacobian, equal to `aom.total_size` |
| `lm_idx`, `res_idx` | column offsets | first landmark column and the residual column |

Two of these rows are traps. The field named `H` inside `MargLinData` is not an information matrix in the default configuration. It is a square root factor $\mathbf{J}_\text{marg}$ satisfying $\mathbf{J}_\text{marg}^T\mathbf{J}_\text{marg} = \mathbf{H}^\ast$, and the companion field `b` is correspondingly a residual rather than an information vector. The boolean `is_sqrt` disambiguates, and it is fixed for the lifetime of the estimator from `config.vio_sqrt_marg`. Reading `H` as a Hessian makes the assembly code of §3.2.3 incomprehensible. The second trap is the positive sign on `b`, noted in §3.1.9.

#### 3.1.12 Efficiency and Sparsity

The performance of the scheme rests on four devices, all visible in the code.

Landmark independence is the structural fact. Since each landmark's block touches no other landmark, the linearisation and the elimination are embarrassingly parallel, and both are dispatched through Intel Threading Building Blocks. Linearisation uses `tbb::parallel_reduce` at `linearization_abs_qr.cpp:233-251` because it must accumulate a scalar error and a validity flag, joining with addition and logical conjunction respectively. Elimination uses the simpler `tbb::parallel_for` at `linearization_abs_qr.cpp:271-279` because it returns nothing.

Shared geometry is computed once, through the relative pose cache of §3.1.3, converting a per observation cost into a per frame pair cost.

Row offsets are precomputed once at construction, in the only sequential loop of the constructor at `linearization_abs_qr.cpp:152-161`, so that the parallel export of §3.1.9 can write each block into a disjoint row range with no synchronisation.

Memory layout is chosen for vectorisation, with row major storage so that a row operation is contiguous, and column padding to a multiple of four.

Against these must be set the one significant inefficiency, which is the full width pose Jacobian of §3.1.5. For a window of ten keyframes and three states the width is around one hundred and five scalars, of which any given observation touches twelve. The stored zeros dominate. The `TODO` in the source proposes a reduced order map as the remedy, and notes that when the reduced and full maps coincide the block wise operations could be skipped entirely.

#### 3.1.13 A Worked Dimensional Example

The abstractions above are easier to hold in mind against concrete numbers. Take a representative inertial window with the default configuration, comprising seven pose only keyframes and three navigation states, observing four hundred landmarks with an average of six observations each, and connected by two preintegrated inertial measurements. Assume a marginalisation prior over the seven keyframe poses and the oldest navigation state.

The column layout follows from §3.1.1. Seven keyframes contribute six columns each and three navigation states contribute fifteen each, so the system has $7 \times 6 + 3 \times 15 = 87$ columns. The prior covers the seven poses and one state, so it occupies the leading $7\times 6 + 15 = 57$ columns, and the remaining thirty columns belong to the two newer states, which the prior says nothing about.

$$N = 87, \qquad N_\text{prior} = 57$$

Each landmark block is then sized by §3.1.5. With six observations a block has $2\times 6 + 3 = 15$ rows. Its column count is the pose width rounded up to a multiple of four, which takes eighty seven to eighty eight, plus three landmark columns and one residual column, giving ninety two, which is divisible by four as the assertion requires.

$$\text{storage} \in \mathbb{R}^{15 \times 92}, \qquad [\underbrace{87}_{\mathbf{J}_p} \mid \underbrace{1}_{\text{pad}} \mid \underbrace{3}_{\mathbf{J}_l} \mid \underbrace{1}_{\mathbf{r}}]$$

Of the $15 \times 87$ pose Jacobian, each observation row pair is nonzero in at most twelve columns, being six for its host and six for its target. The block therefore carries about $12/87$, or fourteen per cent, of its pose entries as genuinely nonzero, and the rest are the structural zeros §3.1.12 identifies as the design's main inefficiency. Across four hundred landmarks the storage is roughly $400 \times 15 \times 92$ scalars, which is about five hundred and fifty thousand doubles, or four and a half megabytes. This is affordable, and it is the number that would grow untenably if the window were enlarged.

After the elimination of §3.1.6 each block exports $15 - 3 = 12$ rows, of which the trailing three are the zero damping rows, so nine rows per landmark carry information. Three degrees of freedom have been consumed by the landmark, which is exactly the count of parameters removed.

The stacked square root system of §3.1.9 then assembles as follows, with the row offsets computed in the order the code computes them.

| Contributor | Rows | Offset | Note |
|---|---|---|---|
| landmarks | $400 \times 12 = 4800$ | 0 | includes 1200 zero damping rows |
| inertial | $2 \times 15 = 30$ | 4800 | fifteen rows per preintegrated measurement |
| pose damping | 0 | 4830 | dead in the current build |
| prior | 57 | 4830 | one row per prior column, since the factor is square |

$$\mathbf{Q}_2^T\mathbf{J}_p \in \mathbb{R}^{4887 \times 87}, \qquad \mathbf{Q}_2^T\mathbf{r} \in \mathbb{R}^{4887}$$

The normal equation exit instead returns an $87 \times 87$ matrix and an $87$ vector, which is the same information compressed by a factor of about fifty six in rows. That compression is exactly where the conditioning of §2.13 is spent.

The marginalisation of §3.3 then partitions those eighty seven columns. Suppose one redundant keyframe is dropped and the oldest navigation state is demoted. The redundant keyframe contributes six columns to the marginalised set, the demoted state contributes its nine velocity and bias columns, and everything else is kept, giving fifteen columns eliminated and seventy two retained. The new prior is a matrix of at most seventy two rows by seventy two columns, and fewer if the eliminated block turns out rank deficient, which the rank tracking of §3.3.3 detects.

$$|\text{idx\_to\_marg}| = 15, \qquad |\text{idx\_to\_keep}| = 72$$

At the next frame that prior becomes the leading seventy two columns of a new eighty seven column system, and the cycle repeats.

### 3.2 Linearisation for VIO

The odometry problem adds inertial factors and the marginalisation prior to the visual factors of §3.1, and is solved by `SqrtKeypointVioEstimator::optimize` at `src/vi_estimator/sqrt_keypoint_vio.cpp:1148-1589`.

#### 3.2.1 What Enters the System

Three families of rows are assembled over the frame variables, beyond the reduced landmark rows already described.

The inertial rows come from `ImuBlock`, one per preintegrated measurement between consecutive states. Each block holds a fifteen by thirty Jacobian and a fifteen vector residual, declared at `imu_block.hpp:19-24`, the thirty columns being the two fifteen wide navigation states at the endpoints. The fifteen rows comprise nine for the preintegrated pose, velocity and rotation residual, whitened by the square root inverse of the preintegration covariance, and six for the two bias random walk residuals weighted by $\sigma_b^{-1}/\sqrt{\Delta t}$. `doc/VIO.md` §3.1.1 and §3.1.2 give the residual and its Jacobians, and `context/vio_residuals_and_priors.md` records the two sign conventions that differ from Forster and colleagues. The First-Estimate Jacobians pattern appears here in the same form as in §3.1.3, at `imu_block.hpp:44-54`.

```cpp
typename PoseVelState<Scalar>::VecN res = imu_meas->residual(
    start_state.getStateLin(), imu_lin_data->g, end_state.getStateLin(),
    start_state.getStateLin().bias_gyro,
    start_state.getStateLin().bias_accel, &d_res_d_start, &d_res_d_end,
    &d_res_d_bg, &d_res_d_ba);

if (start_state.isLinearized() || end_state.isLinearized()) {
  res = imu_meas->residual(
      start_state.getState(), imu_lin_data->g, end_state.getState(),
      start_state.getState().bias_gyro, start_state.getState().bias_accel);
}
```

The damping rows are absent in the current build, for the reason given in §2.9.

The prior rows are the subject of §3.2.3.

#### 3.2.2 Linearisation at Time Zero

At the first frame there is no history to summarise, and the prior serves an entirely different purpose from the one it serves later. It is constructed in the estimator's own constructor at `sqrt_keypoint_vio.cpp:77-114`, with the ordering set alongside the first state at `sqrt_keypoint_vio.cpp:142-146`.

```cpp
marg_data.order.abs_order_map[t_ns] = std::make_pair(0, POSE_VEL_BIAS_SIZE);
marg_data.order.total_size = POSE_VEL_BIAS_SIZE;
marg_data.order.items = 1;
```

The matrix is fifteen by fifteen and diagonal, over that single state, and only ten of its fifteen diagonal entries are made nonzero.

```cpp
if (marg_data.is_sqrt) {
    // prior on position
    marg_data.H.diagonal().template head<3>().setConstant(
        std::sqrt(Scalar(config.vio_init_pose_weight)));
    // prior on yaw
    marg_data.H(5, 5) = std::sqrt(Scalar(config.vio_init_pose_weight));

    // small prior to avoid jumps in bias
    marg_data.H.diagonal().template segment<3>(9).array() =
        std::sqrt(Scalar(config.vio_init_ba_weight));
    marg_data.H.diagonal().template segment<3>(12).array() =
        std::sqrt(Scalar(config.vio_init_bg_weight));
}
```

The square root branch stores $\sqrt{w}$ and the alternative branch stores $w$, which is direct confirmation that the field named `H` holds a square root factor whenever `is_sqrt` is set.

The selection of anchored directions is an observability argument and repays reading against the index ordering of §3.1.1.

| Indices | Block | Weight | Default | Reason |
|---|---|---|---|---|
| 0 to 2 | translation | `vio_init_pose_weight` | 1e8 | global position is unobservable, the gauge must be fixed |
| 5 | yaw | `vio_init_pose_weight` | 1e8 | heading about gravity is unobservable, the gauge must be fixed |
| 3 to 4 | roll and pitch | none | zero | observable from gravity, an artificial prior would bias them |
| 6 to 8 | velocity | none | zero | observable from the inertial and visual measurements |
| 9 to 11 | gyroscope bias | `vio_init_ba_weight` | 1e1 | regularisation only, see the defect below |
| 12 to 14 | accelerometer bias | `vio_init_bg_weight` | 1e2 | regularisation only, see the defect below |

The four anchored gauge directions are precisely the four dimensional nullspace of the visual-inertial information matrix discussed in §2.11. Roll and pitch are left free because the initial orientation is already gravity aligned by `FromTwoVectors(imuData->accel, Vec3::UnitZ())` at `sqrt_keypoint_vio.cpp:245-246`, so they are observable and constraining them would inject information the system does not possess. The vector `marg_data.b` is left at zero, which states that the initial estimate sits exactly at the minimum of the initial prior.

A defect stands in the two bias rows of that table. By the tangent ordering established in §3.1.1, indices nine to eleven are the gyroscope bias and indices twelve to fourteen are the accelerometer bias. The code applies `vio_init_ba_weight`, the accelerometer weight, to indices nine to eleven, and `vio_init_bg_weight`, the gyroscope weight, to indices twelve to fourteen. The two weights are transposed relative to the blocks they name. With the defaults of 1e1 and 1e2 the consequence is that the gyroscope bias is held ten times more loosely than intended and the accelerometer bias ten times more tightly. Both are deliberately weak regularisers whose stated purpose, recorded in the source comment as avoiding jumps in the bias, survives the transposition, so the practical severity is low. The defect is inherited from upstream and is recorded in `context/vio_residuals_and_priors.md`. Anyone tuning these values must target the block actually reached rather than the block named.

A second, parallel structure `nullspace_marg_data` is built alongside with the identical ordering but no prior terms at all, at `sqrt_keypoint_vio.cpp:82-85`. It remains a zero matrix and exists so that `checkMargNullspace` can verify that the true prior has not accumulated information along the unobservable directions.

The contrast with a later prior is complete. At time zero the prior is hand written, diagonal, sparsely populated, derived from no measurement whatever, fixed in size at one fifteen wide block, and positioned at a reference point that has not yet moved. At any later time it is dense, derived entirely from eliminated measurements, variable in size as the window evolves, and anchored at reference points that continue to move under optimisation, which is why it must be re-centred at every use.

#### 3.2.3 Integration of the Marginalisation Prior at Arbitrary Time

This is the mechanism by which the output of the last marginalisation enters the next optimisation, and it is the least obvious part of the system.

The prior is held in a single persistent member `marg_data` of type `MargLinData<Scalar>`, declared at `include/basalt/vi_estimator/sqrt_keypoint_vio.h:247`. It is not a queue. Each call to `marginalize` overwrites it, and the next call to `optimize` reads it. Within a frame the order is `optimize` and then `marginalize`, at `sqrt_keypoint_vio.cpp:1592-1597`, so the prior consumed at frame $k+1$ is the one produced at frame $k$.

The mathematical content of the prior is a quadratic cost over the surviving variables, stated in `doc/VIO.md` §3.1.3 and derived in `doc/Marginalisation.md` Appendix A. In square root form it is a Jacobian and a residual satisfying the relations below, where $\mathbf{H}^\ast$ and $\mathbf{b}^\ast$ are the Schur complement outputs of `doc/Marginalisation.md` §3.2.

$$\mathbf{J}_\text{marg}^T\mathbf{J}_\text{marg} = \mathbf{H}^\ast, \qquad \mathbf{J}_\text{marg}^T\mathbf{r}_\text{marg} = \mathbf{b}^\ast$$

The difficulty is that the prior was linearised at a set of reference points, and those reference points have since moved. Under the First-Estimate Jacobians convention of §2.11 the Jacobian must not be recomputed, but the residual must be brought up to date. Let $\delta$ be the accumulated tangent offset of the surviving variables from their frozen linearisation points, so that the current estimate is $\mathbf{s}_\text{lin}\oplus\delta$. The prior evaluated at a further increment $\mathbf{x}$ from the current estimate is the following.

$$P(\mathbf{x}) = \frac{1}{2}(\|\mathbf{J}_\text{marg}(\delta + \mathbf{x}) + \mathbf{r}_\text{marg})\|^2$$

Linearising this at $\mathbf{x} = 0$, which is what the solver requires, gives a system whose Jacobian is unchanged and whose residual absorbs the offset.

$$P(\mathbf{x}) = \frac{1}{2}(\|\mathbf{J}_\text{marg}\mathbf{x} + (\mathbf{J}_\text{marg}\delta + \mathbf{r}_\text{marg}))\|^2$$

This single equation is the entire mechanism. The rows contributed by the prior are the frozen Jacobian, and the residual contributed is the frozen residual plus the Jacobian acting on the accumulated offset. The corresponding normal equation contributions follow immediately.

$$\mathbf{H} \mathrel{+}= \mathbf{J}_\text{marg}^T\mathbf{J}_\text{marg} = \mathbf{H}^\ast, \qquad \mathbf{b} \mathrel{+}= \mathbf{J}_\text{marg}^T((\mathbf{J}_\text{marg}\delta + \mathbf{r}_\text{marg})) = \mathbf{H}^\ast\delta + \mathbf{b}^\ast$$

The offset itself is collected by `computeDelta` at `src/vi_estimator/ba_base.cpp:315-332`, which walks the prior's own order map and gathers each variable's accumulated tangent offset, asserting on the way that every such variable has indeed been frozen.

```cpp
for (const auto& kv : marg_order.abs_order_map) {
  if (kv.second.second == POSE_SIZE) {
    BASALT_ASSERT(frame_poses.at(kv.first).isLinearized());
    delta.template segment<POSE_SIZE>(kv.second.first) =
        frame_poses.at(kv.first).getDelta();
  } else if (kv.second.second == POSE_VEL_BIAS_SIZE) {
    BASALT_ASSERT(frame_states.at(kv.first).isLinearized());
    delta.template segment<POSE_VEL_BIAS_SIZE>(kv.second.first) =
        frame_states.at(kv.first).getDelta();
  }
}
```

The square root assembly is at `linearization_abs_qr.cpp:604-626` and implements the equation directly.

```cpp
BASALT_ASSERT(marg_lin_data->is_sqrt);
size_t marg_rows = marg_lin_data->H.rows();
size_t marg_cols = marg_lin_data->H.cols();
VecX delta;
estimator->computeDelta(marg_lin_data->order, delta);
if (marg_scaling.rows() > 0) {
  Q2Jp.template block(start_idx, 0, marg_rows, marg_cols) =
      marg_lin_data->H * marg_scaling.asDiagonal();
} else {
  Q2Jp.template block(start_idx, 0, marg_rows, marg_cols) = marg_lin_data->H;
}
Q2r.template segment(start_idx, marg_rows) =
    marg_lin_data->H * delta + marg_lin_data->b;
```

The normal equation assembly is delegated to `BundleAdjustmentBase::linearizeMargPrior` at `src/vi_estimator/ba_base.cpp:407-474`, whose two branches make the square root and squared conventions explicit side by side.

```cpp
if (mld.is_sqrt) {
  abs_H.topLeftCorner(marg_size, marg_size) += mld.H.transpose() * mld.H;
  abs_b.head(marg_size) += mld.H.transpose() * (mld.b + mld.H * delta);
  marg_prior_error = delta.transpose() * mld.H.transpose() *
                     (Scalar(0.5) * mld.H * delta + mld.b);
} else {
  abs_H.topLeftCorner(marg_size, marg_size) += mld.H;
  abs_b.head(marg_size) += mld.H * delta + mld.b;
  marg_prior_error =
      delta.transpose() * (Scalar(0.5) * mld.H * delta + mld.b);
}
```

Three structural questions remain, and each has a definite answer.

The first is how the prior's variable ordering maps onto the current problem's ordering. It does so by identity, with no remapping at all. Both order maps are built by the same rule of frame poses first and frame states second, both in ascending timestamp. Marginalisation never reorders the survivors, and optimisation only ever appends states with newer timestamps. The prior's variables are therefore always the oldest ones and always occupy a contiguous prefix of the columns, so writing the prior into the leading `marg_cols` columns is correct without permutation. The invariant is asserted rather than computed, at `sqrt_keypoint_vio.cpp:1174-1176` and `:1186-1189` on the optimisation path, at `:701` and `:726` on the marginalisation path, and again inside `linearizeMargPrior` at `ba_base.cpp:415-421`. The source comment at `linearization_abs_qr.cpp:375-376` states the assumption plainly, noting that the check for it lives outside the class.

The second is what happens to variables present in the current problem but absent from the prior, which is the case for every state created since the last marginalisation. They occupy columns beyond `marg_lin_data->H.cols()`, and since the prior's block write touches only the leading columns, those columns receive contributions from the visual and inertial factors alone. The prior is silent about them, correctly, since it summarises no measurement that involved them.

The third is what happens to variables that were in the prior but have since been eliminated. They do not appear in the prior's order map at all after the marginalisation that removed them, so no lookup involving them is ever attempted. Their influence survives entirely as coupling between the variables that remain, which is what the Schur complement produced.

One inconsistency between the two assembly paths should be recorded. The square root path honours the column scaling held in `marg_scaling`, whereas the normal equation path asserts that no such scaling is present, at `linearization_abs_qr.cpp:640`.

```cpp
// Scaling not supported ATM
BASALT_ASSERT(marg_scaling.rows() == 0);
```

Since `vio_scale_jacobian` defaults to true, a build that both scaled the Jacobian and called `get_dense_H_b` with a prior present would abort. It does not do so in practice because the odometry loop, which is the only caller of `get_dense_H_b` with a prior, does not invoke `scaleJp_cols`. The assertion is therefore latent rather than live, but it constrains any future change that would enable scaling on that path.

Finally, a commented out block at `linearization_abs_qr.cpp:647-667` preserves the manual form of the same computation, superseded by the delegation above, and is worth quoting because it exhibits the algebraic relationship between the two representations in two lines.

```cpp
//  H.topLeftCorner(marg_size, marg_size) +=
//      marg_lin_data->H.transpose() * marg_lin_data->H;
//  b.head(marg_size) += marg_lin_data->H.transpose() *
//                       (marg_lin_data->H * delta + marg_lin_data->b);
```

#### 3.2.4 The Solve

The odometry path takes the normal equation exit rather than the square root exit. Inside the inner loop it calls `get_dense_H_b`, adds Marquardt damping to the diagonal, and factors with a Cholesky type decomposition, at `sqrt_keypoint_vio.cpp:1341-1380`.

```cpp
lqr->get_dense_H_b(H, b);

VecX Hdiag_lambda = (H.diagonal() * lambda).cwiseMax(min_lambda);
MatX H_copy = H;
H_copy.diagonal() += Hdiag_lambda;

Eigen::LDLT<Eigen::Ref<MatX>> ldlt(H_copy);
inc = ldlt.solve(b);
```

This does square the condition number, as §2.8 notes, and the choice is deliberate. The odometry system is discarded and rebuilt at the next frame, so its conditioning damage does not accumulate, whereas the marginalisation prior persists for the whole run and is therefore kept in square root form. The dense system is small, being $N$ square with $N$ around one hundred, so a dense factorisation is entirely appropriate.

The full per iteration sequence is `linearizeProblem`, then `performQR`, then an inner backtracking loop over damping values in which `get_dense_H_b` is called, the system is solved, the state is backed up, `backSubstitute` recovers the landmark increments and accumulates the predicted decrease, the increment is applied through `applyInc`, and the new cost is compared against the prediction. A successful step shrinks the damping by the Nielsen rule and breaks the inner loop, and a rejected step restores the backup, grows the damping and retries. The linearisation object itself is constructed once per call to `optimize` rather than once per iteration, at `sqrt_keypoint_vio.cpp:1224-1225`, since allocating the landmark blocks is the expensive part.

### 3.3 Linearisation for Marginalisation

Marginalisation invokes the same framework a second time, with different arguments and a different exit, and then eliminates variables from the result. The method is `SqrtKeypointVioEstimator::marginalize` at `sqrt_keypoint_vio.cpp:674-1146`.

#### 3.3.1 What Differs from the Odometry Invocation

Four differences separate the two invocations, and each follows from the different purpose.

The order map is truncated. The loop over `frame_states` breaks at `last_state_to_marg`, at `sqrt_keypoint_vio.cpp:696`, so the sub problem contains only the states old enough to participate in the elimination. Newer states are irrelevant to it and are left untouched.

The landmark set is restricted. The call passes `&kfs_to_marg` as `used_frames` and `&lost_landmaks` as `lost_landmarks`, so only landmarks hosted by a keyframe about to be removed, or landmarks that have lost their observations, are linearised. The odometry path passes null for both and linearises everything.

The inertial set is filtered. Only preintegrated measurements whose endpoints both lie within the truncated order map are included, at `sqrt_keypoint_vio.cpp:437-442`.

The exit is the square root one when the configuration permits. The choice is made at `sqrt_keypoint_vio.cpp:851-855`.

```cpp
if (is_lin_sqrt && marg_data.is_sqrt) {
    lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H, Q2r_or_b);
} else {
    lqr->get_dense_H_b(Q2Jp_or_H, Q2r_or_b);
}
```

The variable name `Q2Jp_or_H` is an honest acknowledgement that one matrix carries two different meanings depending on configuration.

Note that the system handed to the elimination already contains the previous prior, stacked as rows by the mechanism of §3.2.3. Marginalisation therefore composes priors rather than replacing them, and the information summarised at step $k-1$ flows into the prior produced at step $k$ without ever being re-derived.

#### 3.3.2 The Partition into Kept and Marginalised Indices

Every scalar column of the truncated order map is assigned to exactly one of two index sets, at `sqrt_keypoint_vio.cpp:919-948`. The assignment is by variable class and is finer grained than a simple age test.

| Class | Container | Kept | Marginalised |
|---|---|---|---|
| surviving keyframe pose | `frame_poses` | all six | none |
| redundant keyframe pose | `poses_to_marg` | none | all six |
| non-keyframe state | `states_to_marg_all` | none | all fifteen |
| ageing keyframe state | `states_to_marg_vel_bias` | pose, six | velocity and biases, nine |
| boundary state | `last_state_to_marg` | all fifteen | none |

The fourth row is the interesting one and encodes a genuine modelling judgement. A keyframe leaving the recent window retains value as a visual anchor, since landmarks are hosted in it and future frames may still observe them, so its pose is kept. Its velocity and biases have no such value, since no future measurement will constrain them and they only enlarge the state, so they are eliminated. The frame is thereby demoted from a fifteen wide state to a six wide pose, which is exactly the transition from `frame_states` to `frame_poses` that the order map of §3.1.1 assumes.

#### 3.3.3 The Three Elimination Routines

`MargHelper` provides three static routines and the caller dispatches on the pair of booleans that describe the input form and the desired output form, at `sqrt_keypoint_vio.cpp:1019-1039`. All three are in `src/vi_estimator/marg_helper.cpp`.

The routine `marginalizeHelperSqToSq`, at `marg_helper.cpp:42-122`, takes an information matrix and returns one. It permutes the kept indices to the front, then applies the Schur complement of `doc/Marginalisation.md` §3.2 directly.

$$\mathbf{H}^\ast = \mathbf{H}_{kk} - \mathbf{H}_{km}\mathbf{H}_{mm}^{-1}\mathbf{H}_{mk}, \qquad \mathbf{b}^\ast = \mathbf{b}_k - \mathbf{H}_{km}\mathbf{H}_{mm}^{-1}\mathbf{b}_m$$

The inverse is realised as a pseudoinverse through a complete orthogonal decomposition, which is a rank revealing QR, chosen for robustness when the block to be eliminated is near singular. The source records the experimental history of that choice, having tried and rejected both a Cholesky type factorisation and a full pivoting decomposition, and warns explicitly against a singular value based pseudoinverse on grounds of accuracy.

The routine `marginalizeHelperSqToSqrt`, at `marg_helper.cpp:124-255`, performs the same Schur complement and then converts the dense result to square root form through a pivoted Cholesky type factorisation $\mathbf{H}^\ast = \mathbf{P}^T\mathbf{L}\mathbf{D}\mathbf{L}^T\mathbf{P}$, from which the factor and the matching residual follow.

$$\mathbf{J}_\text{marg} = \mathbf{D}^{1/2}\mathbf{L}^T\mathbf{P}, \qquad \mathbf{r}_\text{marg} = \mathbf{D}^{-1/2}\mathbf{L}^{-1}\mathbf{P}\,\mathbf{b}^\ast$$

Negative entries of the diagonal factor, which can arise numerically from a matrix that is positive semi-definite in exact arithmetic, are clamped to zero before the square root is taken, and components whose diagonal entry is below the smallest representable positive value have their residual component set to zero rather than divided.

The routine `marginalizeHelperSqrtToSqrt`, at `marg_helper.cpp:257-347`, is the one the default configuration uses, and it performs no Schur complement and forms no inverse at all. It takes the stacked Jacobian and residual, permutes the columns so that the variables to be eliminated come first, and runs a hand written rank revealing Householder QR in place.

The column order is the reverse of the other two routines, and the reason is worth stating. A QR annihilates columns from the left, so placing the variables to be eliminated first means that the leading reflections consume exactly their column space. Every row below the rank consumed by those columns is thereafter free of them, and is a valid reduced system in the survivors alone. This is the same nullspace projection as §2.7, applied to frame variables rather than to landmarks.

```cpp
Q2Jp.col(k).tail(remainingRows).makeHouseholderInPlace(hCoeff, beta);

if (std::abs(beta) > rank_threshold) {
  Q2Jp.coeffRef(total_rank, k) = beta;
  Q2Jp.bottomRightCorner(remainingRows, remainingCols)
      .applyHouseholderOnTheLeft(Q2Jp.col(k).tail(remainingRows - 1),
                                 hCoeff, tempData + k + 1);
  Q2r.tail(remainingRows)
      .applyHouseholderOnTheLeft(Q2Jp.col(k).tail(remainingRows - 1),
                                 hCoeff, tempData + cols);
  total_rank++;
} else {
  Q2Jp.coeffRef(total_rank, k) = 0;
}
```

The rank tracking is what makes the routine safe against a rank deficient marginalisation block. A column whose remaining norm falls below $\sqrt{\varepsilon_\text{machine}}$ contributes no new row to the triangular factor and is skipped, and the rank actually consumed by the eliminated columns is recorded as `marg_rank` when the loop crosses the boundary between the two column groups. The new prior is then read directly out of the triangular factor.

```cpp
marg_sqrt_H = Q2Jp.block(marg_rank, marg_size, keep_valid_rows, keep_size);
marg_sqrt_b = Q2r.segment(marg_rank, keep_valid_rows);
```

No further factorisation is required, because by the relation of §2.4 a triangular factor of a Jacobian is already a square root of the corresponding information matrix. This is the payoff of carrying the square root through the whole pipeline, and it is why the default path never forms an information matrix from the first residual to the stored prior.

#### 3.3.4 Re-centring and Storage of the New Prior

The prior returned by the elimination is expressed relative to the current estimates of the surviving variables, but a prior must be expressed relative to fixed linearisation points if the convention of §2.11 is to hold. The final step converts between the two, at `sqrt_keypoint_vio.cpp:1096-1121`, and the source states the derivation in its own comment.

```
// Quadratic prior and "delta" of the current state to the original
// linearization point give cost function
//
//    P(x) = 0.5 || J*(delta+x) + r ||^2.
//
// For marginalization this has been linearized at x=0 to give
// linearization
//
//    P(x) = 0.5 || J*x + (J*delta + r) ||^2,
//
// with Jacobian J and residual J*delta + r.
//
// After marginalization, we recover the original form of the
// prior. We are left with linearization (in sqrt form)
//
//    Pnew(x) = 0.5 || Jnew*x + res ||^2.
//
// To recover the original form with delta-independent r, we set
//
//    Pnew(x) = 0.5 || Jnew*(delta+x) + (res - Jnew*delta) ||^2,
//
// and thus rnew = (res - Jnew*delta).
```

```cpp
VecX delta;
computeDelta(marg_data.order, delta);
marg_data.b -= marg_data.H * delta;
```

The operation is the exact inverse of the folding performed in §3.2.3. There the offset was added to produce a residual valid at the current estimate, and here it is subtracted to produce a residual valid at the linearisation point, so that the next optimisation can add it again. The subtraction is by $\mathbf{J}_\text{marg}\delta$ in the square root case and by $\mathbf{H}^\ast\delta$ in the squared case, and the single line is correct in both because the field named `H` holds whichever of the two the configuration selected.

The order of operations matters and is easy to misread. By the time this subtraction runs, `marg_data.H`, `marg_data.b` and `marg_data.order` have already been replaced by the new prior and the new ordering, at `sqrt_keypoint_vio.cpp:1090-1092`, so `computeDelta` gathers the offsets of the survivors only.

```cpp
marg_data.H = marg_H_new;
marg_data.b = marg_b_new;
marg_data.order = marg_order_new;

BASALT_ASSERT(size_t(marg_data.H.cols()) == marg_data.order.total_size);
```

The flag `marg_data.is_sqrt` is never reassigned inside `marginalize`. It is fixed once at construction from `config.vio_sqrt_marg` and holds for the estimator's lifetime, and the three way dispatch of §3.3.3 exists precisely so that whichever routine runs produces output matching that fixed flag.

Two further actions complete the step. The boundary state is frozen by `setLinTrue`, which snapshots its current estimate as its permanent linearisation point and begins the accumulation of `delta`, and this is the moment at which a variable enters the First-Estimate Jacobians regime. Separately, and before the elimination, the full un-eliminated system together with its order map is packaged into a `MargData` object and pushed to an output queue for the mapping layer, as described in `doc/Marginalisation.md` §3.3.2.

---

## 4. Software Abstraction and Framework

### 4.1 Architecture

#### 4.1.1 The Class Hierarchy

The framework is organised into three layers, and keeping them distinct is the key to reading the code.

The strategy layer is the abstract class `LinearizationBase<Scalar, POSE_SIZE>` together with its three concrete descendants. It owns the whole problem, decides how landmarks are eliminated, and produces the assembled system. The estimator interacts with this layer alone.

The block layer comprises `LandmarkBlock<Scalar>` with its single concrete implementation `LandmarkBlockAbsDynamic<Scalar, POSE_SIZE>`, and the unrelated `ImuBlock<Scalar>`. Each object of this layer owns the residuals and Jacobians of exactly one factor group, being one landmark with all of its observations, or one preintegrated inertial measurement. Blocks are independent of one another, which is the property the parallelism exploits.

The elimination layer is the free standing utility `MargHelper<Scalar>`, which is not part of the hierarchy at all. It takes an assembled system and a partition of its columns and returns a reduced system. It knows nothing about landmarks, frames or cameras.

The inheritance tree of the strategy layer is shallow and has exactly three leaves.

```
LinearizationBase<Scalar, POSE_SIZE>            (abstract)
  ├── LinearizationAbsQR<Scalar, POSE_SIZE>     absolute poses, QR landmark elimination   [default]
  ├── LinearizationAbsSC<Scalar, POSE_SIZE>     absolute poses, Schur complement
  └── LinearizationRelSC<Scalar, POSE_SIZE>     relative poses, Schur complement
```

Instantiation is by a static factory rather than by direct construction, at `src/linearization/linearization_base.cpp:58-91`, which switches on the enum carried in the options.

```cpp
switch (options.linearization_type) {
  case LinearizationType::ABS_QR:
    return std::make_unique<LinearizationAbsQR<Scalar, POSE_SIZE>>(...);
  case LinearizationType::ABS_SC:
    return std::make_unique<LinearizationAbsSC<Scalar, POSE_SIZE>>(...);
  case LinearizationType::REL_SC:
    return std::make_unique<LinearizationRelSC<Scalar, POSE_SIZE>>(...);
  default:
    std::cerr << "Could not select a valid linearization." << std::endl;
    std::abort();
}
```

A companion free function reports whether a strategy is a square root one, at `linearization_base.cpp:45-56`, and returns true for `ABS_QR` alone. That single boolean is consulted by the estimator to decide which exit to take from the linearisation and which elimination routine to call, so it is the pivot on which the whole configuration turns.

Two facts about instantiation should be recorded because they bound what the framework can express. Only `POSE_SIZE = 6` is ever instantiated, at `linearization_base.cpp:96-117`, so the template parameter is a documentation device rather than a genuine degree of freedom. And the distinction between visual odometry and visual-inertial odometry is not carried by the type system at all. It is a runtime null check on the inertial data pointer, so the same class serves both and every inertial code path is guarded.

#### 4.1.2 The Interface Contract

The abstract interface is deliberately narrow, comprising six pure virtual methods declared at `include/basalt/linearization/linearization_base.hpp:27-49`.

| Method | Contract |
|---|---|
| `Scalar linearizeProblem(bool* numerically_valid)` | Evaluate all residuals and Jacobians at the current estimate. Returns the total objective value. Sets the flag false if any landmark was numerically degenerate. |
| `void performQR()` | Eliminate the landmarks in place. A no-op in the two Schur complement strategies, which eliminate during linearisation instead. |
| `void get_dense_H_b(MatX& H, VecX& b) const` | Return the assembled normal equations over the frame variables, of size $N\times N$ and $N$. |
| `void get_dense_Q2Jp_Q2r(MatX& Q2Jp, VecX& Q2r) const` | Return the assembled system in square root form, with $N$ columns and a row count that depends on the active contributors. |
| `Scalar backSubstitute(const VecX& pose_inc)` | Recover and apply the landmark increments given the solved frame increment. Returns the predicted decrease in the quadratic model. |
| `void log_problem_stats(ExecutionStats& stats) const` | Reporting hook. |

Five further methods are implemented on `LinearizationAbsQR` but are commented out of the base interface at `linearization_base.hpp:33-45`, namely `setPoseDamping`, `hasPoseDamping`, `getJp_diag2`, `scaleJl_cols`, `scaleJp_cols` and `setLandmarkDamping`. A caller holding a base class pointer therefore cannot reach them. The reason is visible in the two Schur complement strategies, where every one of them aborts with a not implemented assertion, since per landmark damping and per landmark column scaling are meaningless once the landmarks have already been eliminated. Removing them from the interface rather than leaving them to abort at runtime is the safer choice, and the consequence for the reader is that the odometry loop cannot exercise them through the abstraction.

Two named methods that a reader might expect do not exist anywhere in the tree, and a search for `numScalarsLandmark` or `getLandmarkBlocks` returns nothing. The nearest real counterpart is `LandmarkDatabase::numLandmarks`.

The reporting hook is presently inert. `LinearizationAbsQR::log_problem_stats` at `src/linearization/linearization_abs_qr.cpp:178-181` has an empty body, as do the corresponding overrides on both Schur complement classes. Every statistic an engineer actually sees, being the counts of cameras, landmarks and observations together with the timings of each stage, is recorded at the estimator level instead, in `optimize` at `sqrt_keypoint_vio.cpp:1203-1205` and following. The hook is an extension point that has not been taken up, and a reader looking for linearisation diagnostics should not expect it to supply any.

#### 4.1.3 The Landmark Block State Machine

`LandmarkBlockAbsDynamic` enforces an explicit lifecycle through an enumeration declared at `include/basalt/linearization/landmark_block.hpp:46-52`, and every stateful method asserts its precondition. The machine is worth tabulating because it is the clearest statement of the intended call order.

| Method | Required state on entry | State on exit |
|---|---|---|
| `allocateLandmark` | `Uninitialized` | `Allocated` |
| `linearizeLandmark` | `Allocated`, `NumericalFailure`, `Linearized` or `Marginalized` | `Linearized`, or `NumericalFailure` |
| `addJp_diag2` | `Linearized` | unchanged |
| `scaleJl_cols` | `Linearized` | unchanged |
| `performQR` | `Linearized` | `Marginalized` |
| `scaleJp_cols` | `Marginalized` | unchanged |
| `setLandmarkDamping` | `Marginalized` | unchanged |
| `backSubstitute` | `Marginalized` | unchanged |

The ordinary path is therefore a linear chain from `Uninitialized` through `Allocated` and `Linearized` to `Marginalized`, with `NumericalFailure` as a side branch.

Three properties of the machine carry real meaning.

Only `linearizeLandmark` accepts more than one entry state, and it accepts every state except `Uninitialized`. This is what permits the outer loop to re-linearise a block that was already reduced in a previous iteration, and it is safe because the method begins by zeroing the storage matrix, so the in place triangularisation of the previous iteration cannot leak into the next.

A block in `NumericalFailure` is accepted by `linearizeLandmark` and by nothing else. It cannot reach `performQR` until a subsequent linearisation succeeds, which is the mechanism that prevents a degenerate landmark from corrupting the reduction.

The two scaling methods sit on opposite sides of the reduction, with landmark column scaling requiring `Linearized` and pose column scaling requiring `Marginalized`. This encodes the ordering argument of §2.14, since landmark columns must be scaled before they are annihilated and pose columns can only be scaled once the rows that describe them are the reduced ones.

#### 4.1.4 The Input Data

The linearisation reads four structures and owns none of them.

The landmark database, `LandmarkDatabase<Scalar>` in `include/basalt/vi_estimator/landmark_database.h`, holds two containers. The first maps a landmark identifier to a `Keypoint<Scalar>`, which carries the three parameters, being a two component bearing direction and a scalar inverse distance, together with the identifier of its host view and a map from every observing view to the observed pixel. The second is an index of the observation structure, and its nesting is the one the constructor walks.

```cpp
Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar>> kpts;

std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<KeypointId>>>
    observations;
```

The nesting is host view, then target view, then the set of landmarks that pair observes. A `TimeCamId` is a frame timestamp paired with a camera index, so a stereo pair contributes two distinct views per frame. The host is the view in whose frame the landmark's three parameters are expressed, fixed when the landmark is created and never changed. A target is any view holding an observation of it, and a landmark is normally observed in its own host view, in which case the host equals the target and the relative pose is the identity with zero Jacobians.

The distinction matters for a reason that surfaces during marginalisation. Because the parameterisation is anchored to the host, a landmark cannot survive the removal of its host frame, and `removeKeyframes` at `src/vi_estimator/landmark_database.cpp:69-96` drops such landmarks outright. A landmark that merely loses some target observations survives unless fewer than `min_num_obs`, which is two, remain.

The order map, `AbsOrderMap`, supplies the column layout described in §3.1.1.

The inertial data, `ImuLinData<Scalar>`, supplies gravity, the two bias random walk weights and a map from timestamp to preintegrated measurement. A null pointer here is what makes the problem purely visual.

The marginalisation prior, `MargLinData<Scalar>`, supplies the factor, the residual, the ordering and the flag that says which of the two representations is in use.

#### 4.1.5 The Three Strategies Compared

Although only the default is exercised by any shipped configuration, the alternatives illuminate the design by contrast, and the batch configuration used for the ICCV paper exercises all of them.

| Aspect | `LinearizationAbsQR` | `LinearizationAbsSC` | `LinearizationRelSC` |
|---|---|---|---|
| Pose parameterisation | absolute throughout | absolute throughout | relative per host, projected to absolute afterwards |
| Landmark elimination | in place QR on a stacked Jacobian | analytic three by three Schur complement | the same Schur complement, computed in the relative frame first |
| When elimination happens | in `performQR` | during `linearizeProblem` | during `linearizeProblem` |
| Is `performQR` meaningful | yes, it does the work | no, empty body | no, empty body |
| Is a dense $\mathbf{H}$ formed | only if `get_dense_H_b` is called | always | always |
| What `get_dense_Q2Jp_Q2r` returns | a genuine orthogonal projection | a synthetic square root of $\mathbf{H}$ by LDLT | the same synthetic square root |
| Per block unit | `LandmarkBlock`, one per landmark | `AbsLinData`, one per host | `RelLinData`, one per host |
| Damping and scaling methods | implemented | abort as not implemented | abort as not implemented |

Two observations are worth drawing out.

The name `get_dense_Q2Jp_Q2r` is honest only in the QR strategy. In both Schur complement strategies the method forms the dense normal equations first and then factors them with LDLT to synthesise a square root, at `linearization_abs_sc.cpp:253-291`. The result satisfies the same algebraic contract, so the marginalisation code can consume it, but there is no orthogonal projection and no $\mathbf{Q}_2$ anywhere in it. A reader tracing that name across the three classes should expect the same interface and quite different mathematics.

The relative strategy differs from the absolute one only in when the chain rule from relative to absolute poses is applied. The absolute strategy applies it per observation, immediately, and eliminates landmarks in absolute coordinates. The relative strategy eliminates landmarks in the relative frame first, producing a small dense matrix per host, and projects that matrix into absolute coordinates afterwards through the same two Jacobians. Both use `computeRelPose` and both arrive at the same answer.

The shipped configurations all pin `ABS_QR` explicitly, at `data/euroc_config.json:11` and its siblings, which matches the default rather than overriding it. The one place the default is genuinely varied is the batch sweep at `data/iccv21/basalt_batch_config.toml:370-378`, which compares a non square root Schur complement, a square root Schur complement and the square root QR path against one another. The relative strategy appears in no shipped configuration and only in `test/src/test_linearization.cpp`.

#### 4.1.6 Accumulators

Two helper types exist to receive scattered contributions.

`DenseAccumulator<Scalar>`, at `include/basalt/optimization/accumulator.h:64-138`, wraps a dense matrix and vector behind bounds checked block addition methods, `addH` and `addB`, templated on the block dimensions so that the sizes are known at compile time. It exists because contributions arrive as small fixed size blocks at arbitrary offsets, and the templated interface both documents the block size at each call site and lets the compiler emit fixed size arithmetic rather than dynamic loops. A sparse variant, `SparseHashAccumulator`, exists for the mapping layer, which has a much larger and sparser system.

`BlockDiagonalAccumulator`, at `include/basalt/linearization/block_diagonal.hpp`, accumulates a block diagonal approximation of the information matrix for use as a preconditioner. It is live only in the QR family and the corresponding landmark block method aborts as unimplemented, so it is presently dead in the shipped path.

An asymmetry in the inertial block is worth noting because it looks like an oversight and is documented as a deliberate one. `ImuBlock` implements accumulation into a `DenseAccumulator` but not into a bare matrix and vector, so `LinearizationAbsQR::add_dense_H_b_imu` creates a throwaway accumulator, lets the inertial blocks scatter into it, and then adds its contents into the caller's matrices. The source calls this a workaround at `linearization_abs_qr.cpp:449-450`.

#### 4.1.7 The Threading Model

The framework is parallelised with Intel Threading Building Blocks, and the division between parallel and sequential work follows the block independence of §4.1.1 exactly. All parallelism lives in the strategy layer translation unit, and the estimator itself contains no parallel construct at all.

| Operation | Mode | Anchor |
|---|---|---|
| relative pose cache | sequential | `linearization_abs_qr.cpp:192-224` |
| landmark block allocation | `parallel_for` | `:135-149` |
| landmark linearisation | `parallel_reduce`, summing error and conjoining validity | `:233-251` |
| inertial linearisation | sequential | `:255-259` |
| landmark elimination | `parallel_for` | `:271-279` |
| back substitution over landmarks | `parallel_reduce` with `std::plus` | `:296-305` |
| back substitution over inertial blocks | sequential | `:306-309` |
| column norm accumulation | `parallel_reduce` with a hand written reductor | `:325-358` |
| landmark scaling and damping | `parallel_for` | `:392-400`, `:471-479` |
| dense assembly, landmark part | `parallel_reduce` with a reductor | `:483-518`, `:537-578` |
| dense assembly, inertial, damping and prior parts | sequential | `:494-500`, `:671-694` |

The rule behind the table is simply the count of items. Landmark blocks number in the hundreds or thousands and are worth distributing. Inertial blocks number as many as there are states in the window, which is a handful, so the synchronisation would cost more than the work. The relative pose cache is sequential because it writes into a shared hash map.

The reductions are not merely parallel loops, since each carries a genuine reduction operator. The linearisation reduces a pair of a scalar error and a boolean validity flag, joining by addition and logical conjunction respectively, so a single degenerate landmark anywhere in the problem propagates its invalidity to the caller. The dense assembly reduces whole matrices by addition, which is correct because every contribution is additive.

One further scheduling property matters for correctness. The row offsets at which each landmark block writes into the stacked system are precomputed in the only sequential loop of the constructor, at `linearization_abs_qr.cpp:152-161`, so the parallel export writes into disjoint ranges and needs no synchronisation whatever.

#### 4.1.8 A Worked Example

The following is the minimal sequence that takes a window of states and a landmark database to a solved increment. It is written out because the individual calls are documented above but their order is not obvious, and because the same five calls appear, with different arguments, in both of the real call sites.

```cpp
// 1. Describe the layout of the unknowns. Frame poses first, then frame
//    states, both in ascending timestamp order.
AbsOrderMap aom;
for (const auto& kv : frame_poses) {
  aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_SIZE);
  aom.total_size += POSE_SIZE;
  aom.items++;
}
for (const auto& kv : frame_states) {
  aom.abs_order_map[kv.first] =
      std::make_pair(aom.total_size, POSE_VEL_BIAS_SIZE);
  aom.total_size += POSE_VEL_BIAS_SIZE;
  aom.items++;
}

// 2. Configure the strategy. The two landmark-block settings must agree with
//    the estimator's own, and the constructor asserts that they do.
typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
lqr_options.lb_options.huber_parameter = huber_thresh;
lqr_options.lb_options.obs_std_dev = obs_std_dev;
lqr_options.linearization_type = config.vio_linearization_type;

// 3. Gather the inertial factors. Pass nullptr instead for a visual-only
//    problem, which is the only thing that distinguishes VO from VIO.
ImuLinData<Scalar> ild = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};
for (const auto& kv : imu_meas) ild.imu_meas[kv.first] = &kv.second;

// 4. Construct. This is the expensive step, since it allocates one landmark
//    block per landmark and one inertial block per measurement.
auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
    this, aom, lqr_options, &marg_data, &ild);

// 5. Evaluate residuals and Jacobians at the current estimate.
bool valid;
Scalar error = lqr->linearizeProblem(&valid);

// 6. Eliminate the landmarks in place.
lqr->performQR();

// 7. Take one of the two exits.
MatX H; VecX b;
lqr->get_dense_H_b(H, b);          // normal equations, N by N
// or: lqr->get_dense_Q2Jp_Q2r(Q2Jp, Q2r);   // square-root form

// 8. Damp and solve.
H.diagonal() += (H.diagonal() * lambda).cwiseMax(min_lambda);
Eigen::LDLT<Eigen::Ref<MatX>> ldlt(H);
VecX inc = ldlt.solve(b);
inc = -inc;                        // b carries a positive sign in this code

// 9. Recover the landmark increments and the predicted cost decrease, then
//    apply the frame increments on the manifold.
Scalar l_diff = lqr->backSubstitute(inc);
for (auto& [frame_id, state] : frame_poses)
  state.applyInc(inc.template segment<POSE_SIZE>(
      aom.abs_order_map.at(frame_id).first));
for (auto& [frame_id, state] : frame_states)
  state.applyInc(inc.template segment<POSE_VEL_BIAS_SIZE>(
      aom.abs_order_map.at(frame_id).first));
```

Four points about this sequence are easy to get wrong.

The object is constructed once and reused across the iterations of the damping loop, because construction allocates every block and is the expensive part. Only steps five and six must be repeated when the linearisation point moves, and only steps seven through nine when only the damping changes.

Steps five and six must both precede either exit. Calling `get_dense_H_b` on a strategy whose blocks are still in the `Linearized` state will trip the state assertion of §4.1.3.

The increment must be negated, because the assembled vector carries $\mathbf{J}^T\mathbf{r}$ rather than its negation, as §3.1.9 notes.

The increment must be handed to `backSubstitute` before it is applied to the states, because the back substitution reads the stored Jacobians and expects the linearisation point to be unchanged.

### 4.2 Implementation for VIO

The odometry path is `SqrtKeypointVioEstimator::optimize`, at `src/vi_estimator/sqrt_keypoint_vio.cpp:1148-1589`, and it is the worked example above with a damping loop wrapped around it.

The order map is built exactly as in the example, and each entry is cross checked against the prior's own ordering as it is written, which is the enforcement of the prefix invariant discussed in §3.2.3.

```cpp
for (const auto& kv : frame_poses) {
    aom.abs_order_map[kv.first] =
        std::make_pair(aom.total_size, POSE_SIZE);

    // Check that we have the same order as marginalization
    BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) ==
                  aom.abs_order_map.at(kv.first));

    aom.total_size += POSE_SIZE;
    aom.items++;
}
```

The strategy object is created once per call, with the prior and the inertial data supplied and the three filtering arguments left at their defaults, so every landmark in the database is linearised.

```cpp
lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
    this, aom, lqr_options, &marg_data, &ild);
```

The loop then has two levels. The outer level re-linearises, and the inner level retries the solve with increasing damping until a step is accepted.

```cpp
// outer, once per linearisation point
error_total = lqr->linearizeProblem(&numerically_valid);
lqr->performQR();

// inner, once per damping trial
lqr->get_dense_H_b(H, b);
VecX Hdiag_lambda = (H.diagonal() * lambda).cwiseMax(min_lambda);
MatX H_copy = H;
H_copy.diagonal() += Hdiag_lambda;
Eigen::LDLT<Eigen::Ref<MatX>> ldlt(H_copy);
inc = ldlt.solve(b);
```

The step is then tried and either kept or reverted. The backup is taken unconditionally before every trial, and covers the frame states, the frame poses and the landmark database together.

```cpp
inline void backup() {
    for (auto& kv : this->frame_states) kv.second.backup();
    for (auto& kv : this->frame_poses) kv.second.backup();
    this->lmdb.backup();
}
```

Two details of this loop are worth flagging for anyone modifying it.

The damping is applied to the assembled dense matrix rather than through the strategy's own damping interface. The call to `setPoseDamping` is commented out at `sqrt_keypoint_vio.cpp:1314-1319`, so the square root pose damping rows described in §2.9 are never generated and the row offset reserved for them in the stacked assembly is always empty.

Outlier filtering is not performed inside this loop. `BundleAdjustmentBase::filterOutliers` exists at `src/vi_estimator/ba_base.cpp:270-309` and is used by the mapping layer, but at `sqrt_keypoint_vio.cpp:1566` there is only a comment noting that it is not called here. Robustness within the odometry loop therefore rests entirely on the Huber weighting of §2.5, and a grossly wrong observation is down weighted but never removed. This is a known gap rather than a defect, and it is recorded here so that a reader does not assume a rejection mechanism that is absent.

The pure visual twin, `SqrtKeypointVoEstimator` in `src/vi_estimator/sqrt_keypoint_vo.cpp`, uses the same framework and differs in four respects worth recording because they show which parts of the design are inertial specific.

It never supplies inertial data, passing null at `sqrt_keypoint_vo.cpp:1049-1050` and explicitly at `:758-759`, so no inertial block is ever created.

Its order map contains only pose blocks, and `frame_states` is asserted empty at `sqrt_keypoint_vo.cpp:1026`. Every block is therefore six wide and the fifteen wide case never arises.

Its order map is seeded from the prior's ordering and extended, rather than being rebuilt from scratch and cross checked. This is the more defensive of the two constructions, since it cannot produce a layout that disagrees with the prior.

Its initial prior weights all six pose degrees of freedom uniformly, at `sqrt_keypoint_vo.cpp:86-89`, rather than distinguishing the four unobservable directions from the two observable ones. The distinction cannot be made without an inertial gravity reference, so a pure visual system has six unobservable directions rather than four, plus scale in the monocular case, and anchoring all six is correct.

### 4.3 Implementation for Marginalisation

The marginalisation path is `SqrtKeypointVioEstimator::marginalize`, at `sqrt_keypoint_vio.cpp:674-1146`. It uses the same framework, and the differences are entirely in the arguments and the exit.

The order map is truncated at the boundary state, and the classification of each frame into the categories of §3.3.2 happens in the same loop that builds it.

```cpp
for (const auto& kv : frame_states) {
    if (kv.first > last_state_to_marg) break;

    if (kv.first != last_state_to_marg) {
        if (kf_ids.count(kv.first) > 0) {
            states_to_marg_vel_bias.emplace(kv.first);
        } else {
            states_to_marg_all.emplace(kv.first);
        }
    }
    ...
}
```

The strategy object is created with all three filtering arguments supplied, so that only the landmarks about to leave the state are linearised.

```cpp
auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
    this, aom, lqr_options, &marg_data, &ild, &kfs_to_marg,
    &lost_landmaks, last_state_to_marg);

lqr->linearizeProblem();
lqr->performQR();

if (is_lin_sqrt && marg_data.is_sqrt) {
    lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H, Q2r_or_b);
} else {
    lqr->get_dense_H_b(Q2Jp_or_H, Q2r_or_b);
}
```

The assembled system already contains the previous prior, stacked as rows by the mechanism of §3.2.3, so the elimination composes priors rather than replacing them.

The partition of §3.3.2 is then built and handed with the system to the appropriate elimination routine, the three way dispatch matching the two booleans that describe the input and output forms.

```cpp
if (is_lin_sqrt && marg_data.is_sqrt) {
    MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
        Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
        marg_b_new);
} else if (marg_data.is_sqrt) {
    MargHelper<Scalar>::marginalizeHelperSqToSqrt(...);
} else {
    MargHelper<Scalar>::marginalizeHelperSqToSq(...);
}
```

The relationship between the two processes can now be stated compactly. Both construct the same class over the same estimator with the same option struct, and both call the same three methods in the same order. They differ in four arguments and one exit.

| | Odometry | Marginalisation |
|---|---|---|
| order map | full window | truncated at the boundary state |
| landmark filter | none, all landmarks | hosted by a departing keyframe, or lost |
| inertial filter | none, all measurements | both endpoints must lie in the truncated map |
| exit | `get_dense_H_b` | `get_dense_Q2Jp_Q2r` when both flags permit |
| consumer | LDLT solve, then back substitution | `MargHelper`, then storage as the new prior |

The prior is the channel between them. Odometry consumes it as extra rows and never modifies it. Marginalisation consumes it the same way, eliminates variables from the combined system, and writes the result back. Because the odometry step precedes the marginalisation step within a frame, the prior consumed at frame $k+1$ is the one produced at frame $k$, and the information from every measurement ever eliminated flows forward through this single object without ever being re-derived.

---

## 5. Key Classes and Interfaces

This section is the API appendix. It follows the table format of `doc/VIO.md` §9 rather than the numbered list format of `doc/Marginalisation.md` §3.3.1, because the classes here expose many small fields and the table is the more compact of the two. Entries are ordered from the strategy layer outward to the supporting types.

### 5.1 `LinearizationBase<Scalar, POSE_SIZE>`

- **Header:** `include/basalt/linearization/linearization_base.hpp`
- **Source:** `src/linearization/linearization_base.cpp`
- **Purpose:** Abstract strategy interface for linearising a windowed estimation problem and eliminating its landmarks. Owns nothing and reads the estimator's state through a back pointer.
- **Inheritance:** Base of `LinearizationAbsQR`, `LinearizationAbsSC` and `LinearizationRelSC`. Instantiated for `POSE_SIZE = 6` only, for `Scalar` in `{float, double}`.

| Field / Method | Signature | Role |
|---|---|---|
| `Options` | struct | Holds `lb_options`, forwarded to every landmark block, and `linearization_type`, the enum the factory switches on. |
| `create(...)` | static, eight parameters | Factory. Selects the concrete class from `options.linearization_type`. Aborts on an unrecognised value. |
| `linearizeProblem(bool*)` | pure virtual, returns `Scalar` | Evaluates all residuals and Jacobians. Returns the total objective. Sets the flag false on any numerical degeneracy. |
| `performQR()` | pure virtual | Eliminates landmarks in place. Empty in both Schur complement strategies. |
| `get_dense_H_b(MatX&, VecX&)` | pure virtual, const | Returns the normal equations over the frame variables, $N \times N$ and $N$. Note $\mathbf{b}$ carries a positive sign. |
| `get_dense_Q2Jp_Q2r(MatX&, VecX&)` | pure virtual, const | Returns the system in square root form, $N$ columns. |
| `backSubstitute(const VecX&)` | pure virtual, returns `Scalar` | Recovers and applies landmark increments. Returns the model's predicted decrease. |
| `log_problem_stats(ExecutionStats&)` | pure virtual, const | Reporting hook. Empty in all three implementations. |
| `isLinearizationSqrt(type)` | free function | True for `ABS_QR` alone. Governs which exit and which elimination routine the estimator uses. |

Five methods are implemented on `LinearizationAbsQR` but commented out of this interface at `linearization_base.hpp:33-45`, namely `setPoseDamping`, `hasPoseDamping`, `getJp_diag2`, `scaleJl_cols`, `scaleJp_cols` and `setLandmarkDamping`. They are unreachable through a base pointer.

### 5.2 `LinearizationAbsQR<Scalar, POSE_SIZE>`

- **Header:** `include/basalt/linearization/linearization_abs_qr.hpp`
- **Source:** `src/linearization/linearization_abs_qr.cpp`
- **Purpose:** The default strategy. Absolute pose parameterisation with QR based landmark elimination, keeping the problem in square root form.
- **Inheritance:** `LinearizationBase<Scalar, POSE_SIZE>`.

| Field / Method | Type or signature | Role |
|---|---|---|
| `landmark_ids` | `std::vector<KeypointId>` | Landmarks selected for this pass. Parallel to `landmark_blocks`. |
| `landmark_blocks` | `std::vector<LandmarkBlockPtr>` | One reduction unit per landmark. |
| `imu_blocks` | `std::vector<ImuBlockPtr>` | One per preintegrated measurement. Empty for a visual-only problem. |
| `landmark_block_idx` | `std::vector<size_t>` | Starting row of each block in the stacked system. Computed once, sequentially. |
| `num_rows_Q2r` | `size_t` | Total reduced rows contributed by the landmark blocks. |
| `relative_pose_lin` | `aligned_unordered_map<pair<TimeCamId,TimeCamId>, RelPoseLin>` | The relative pose cache of §3.1.3. Computed once per pass, shared by every landmark on that view pair. |
| `aom` | `const AbsOrderMap&` | Column layout. Bound by reference. |
| `marg_lin_data` | `const MargLinData<Scalar>*` | The prior, or null. |
| `imu_lin_data` | `const ImuLinData<Scalar>*` | Inertial factors, or null. The only thing distinguishing VO from VIO. |
| `marg_scaling` | `VecX` | Column scaling applied to the prior. Set exactly once, asserted. |
| `pose_damping_diagonal`, `..._sqrt` | `Scalar` | Damping value and its square root. Dead in the current build. |
| `num_cameras` | `size_t` | `frame_poses.size()` at construction. |
| `linearizeProblem(bool*)` | override | Relative pose cache, then parallel landmark linearisation, then sequential inertial, then the prior's error. |
| `performQR()` | override | Parallel dispatch to `LandmarkBlock::performQR`. |
| `get_dense_Q2Jp_Q2r(...)` | override, const | Stacks landmarks, inertial, damping and prior rows in that fixed order. |
| `get_dense_H_b(...)` | override, const | Accumulates the same contributions into $N \times N$ normal equations. |
| `backSubstitute(const VecX&)` | override | Parallel over landmarks, sequential over inertial blocks, then the prior's model cost change. |
| `getJp_diag2()` | concrete | Squared column norms across every contributor, for scaling. Excludes damping deliberately. |
| `scaleJl_cols()`, `scaleJp_cols(...)` | concrete | Column scaling of landmark and pose columns respectively. |
| `setPoseDamping`, `setLandmarkDamping` | concrete | Damping. The pose variant is dead, its call site commented out. |

The constructor parameter `last_state_to_marg` is accepted and discarded at `linearization_abs_qr.cpp:67`.

### 5.3 `LinearizationAbsSC` and `LinearizationRelSC`

- **Headers:** `include/basalt/linearization/linearization_abs_sc.hpp`, `linearization_rel_sc.hpp`
- **Sources:** `src/linearization/linearization_abs_sc.cpp`, `linearization_rel_sc.cpp`
- **Purpose:** Alternative strategies using the classical Schur complement. Not selected by any shipped configuration except the paper's batch sweep.
- **Inheritance:** `LinearizationBase<Scalar, POSE_SIZE>`.

| Field / Method | Role |
|---|---|
| `ald_vec` / `rld_vec` | `AbsLinData` or `RelLinData`, one per host keyframe, replacing the per landmark block of the QR strategy. |
| `performQR()` | Empty body. Landmarks are already eliminated during `linearizeProblem`. |
| `get_dense_H_b(...)` | The natural output. Always forms a dense matrix. |
| `get_dense_Q2Jp_Q2r(...)` | Forms the dense matrix and factors it with LDLT to synthesise a square root. Not an orthogonal projection despite the name. |
| `getJp_diag2`, `scaleJl_cols`, `scaleJp_cols`, `setLandmarkDamping`, `get_dense_Q2Jp_Q2r_pose_damping` | All abort with a not implemented assertion. |

`LinearizationRelSC` differs from `LinearizationAbsSC` only in eliminating landmarks in the relative frame first and projecting to absolute coordinates afterwards, through `linearizeRel` at `src/vi_estimator/sc_ba_base.cpp:501-539` followed by `linearizeAbs`.

### 5.4 `LandmarkBlock<Scalar>`

- **Header:** `include/basalt/linearization/landmark_block.hpp`
- **Source:** `src/linearization/landmark_block.cpp`
- **Purpose:** Abstract interface for the per landmark reduction unit.
- **Inheritance:** Base of `LandmarkBlockAbsDynamic`, which is its only implementation.

| Field / Method | Role |
|---|---|
| `Options` | `obs_std_dev`, `huber_parameter`, `jacobi_scaling_eps`, `use_householder`, `use_valid_projections_only`. |
| `State` | `Uninitialized`, `Allocated`, `NumericalFailure`, `Linearized`, `Marginalized`. See §4.1.3. |
| `createLandmarkBlock<POSE_SIZE>()` | static factory, returns a `LandmarkBlockAbsDynamic`. |
| `allocateLandmark(...)` | Sizes and allocates the storage matrix. |
| `linearizeLandmark()` | Fills it. Returns this landmark's contribution to the objective. |
| `performQR()` | Eliminates the landmark in place. |
| `backSubstitute(...)` | Recovers the landmark increment and accumulates the predicted cost decrease. |
| `numQ2rows()` | Reduced row count, equal to `num_rows - 3`. |
| `get_rel_permutation`, `compute_rel_permutation` | Declared pure virtual, abort in the implementation. Unused. |

### 5.5 `LandmarkBlockAbsDynamic<Scalar, POSE_SIZE>`

- **Header:** `include/basalt/linearization/landmark_block_abs_dynamic.hpp`
- **Source:** N/A, defined inline in the header.
- **Purpose:** The single dense storage matrix and every operation upon it. The central data structure of the framework.
- **Inheritance:** `LandmarkBlock<Scalar>`.

| Field / Method | Type or signature | Role |
|---|---|---|
| `storage` | row major dense matrix | Holds $[\mathbf{J}_p \mid \text{pad} \mid \mathbf{J}_l \mid \mathbf{r}]$ for every observation of one landmark. See §3.1.5 for the exact layout. |
| `padding_idx` | `size_t` | Width of the pose Jacobian, equal to `aom.total_size`. |
| `lm_idx`, `res_idx` | `size_t` | First landmark column and the residual column. |
| `num_rows`, `num_cols` | `size_t` | $2n_\text{obs} + 3$ and a multiple of four respectively. |
| `pose_lin_vec` | `std::vector<const RelPoseLin*>` | Per observation pointer into the shared relative pose cache, or null if the target left the window. |
| `res_idx_by_abs_pose_` | `map<FrameId, set<size_t>>` | Sparsity index, mapping a frame to the observation rows touching it. |
| `damping_rotations` | `std::vector<JacobiRotation>` | The six Givens rotations that fold damping into the triangular factor, stored so they can be undone. |
| `Jl_col_scale` | `Vec3` | Landmark column scaling, undone on the recovered increment. |
| `linearizeLandmark()` | override | Whitens and writes the residual and both Jacobians, chain ruled into the host and target columns. |
| `performQR()` | override | Three Householder reflections, or Givens if configured, applied across the whole matrix. |
| `setLandmarkDamping(Scalar)` | override | Writes $\sqrt{\lambda}$ into the reserved rows and folds them in with six reversible rotations. |
| `backSubstitute(...)` | override | Three by three triangular solve, plus the model cost change. Clamps the inverse distance at zero. |
| `get_dense_Q2Jp_Q2r(...)` | override | Copies rows $[3, \text{num\_rows})$ of the pose columns and the residual column. |
| `add_dense_H_b(MatX&, VecX&)` | override | Adds $\mathbf{J}^T\mathbf{J}$ and $\mathbf{J}^T\mathbf{r}$ over the same rows. |
| `addJp_diag2(VecX&)` | override | Squared column norms, using the sparsity index. |

### 5.6 `ImuBlock<Scalar>`

- **Header:** `include/basalt/linearization/imu_block.hpp`
- **Source:** N/A, defined inline in the header.
- **Purpose:** One preintegrated inertial factor between consecutive navigation states, together with the two bias random walk residuals.
- **Inheritance:** None.

| Field / Method | Type or signature | Role |
|---|---|---|
| `Jp` | $15 \times 30$ | Jacobian with respect to the two endpoint states. |
| `r` | $15$ | Nine preintegration rows plus six bias random walk rows. |
| `linearizeImu(frame_states)` | returns `Scalar` | Jacobians at the frozen linearisation points, residual recomputed at the current estimate when either endpoint is frozen. |
| `add_dense_Q2Jp_Q2r(...)` | | Writes the two fifteen column halves at the endpoint offsets, in a private fifteen row slice. |
| `add_dense_H_b(DenseAccumulator&)` | | Scatters the four fifteen by fifteen normal equation blocks. |
| `backSubstitute(...)` | | Accumulates this factor's predicted cost decrease. |

### 5.7 `MargHelper<Scalar>`

- **Header:** `include/basalt/vi_estimator/marg_helper.h`
- **Source:** `src/vi_estimator/marg_helper.cpp`
- **Purpose:** Static utilities that eliminate a set of columns from an assembled system. Knows nothing of landmarks, frames or cameras.
- **Inheritance:** None.

| Method | Input and output form | Mechanism |
|---|---|---|
| `marginalizeHelperSqToSq` | information matrix in, information matrix out | Permutes kept columns first, then the Schur complement with a pseudoinverse by complete orthogonal decomposition. |
| `marginalizeHelperSqToSqrt` | information matrix in, square root out | The same Schur complement, then a pivoted LDLT converted to $\mathbf{D}^{1/2}\mathbf{L}^T\mathbf{P}$. Negative diagonal entries clamped to zero. |
| `marginalizeHelperSqrtToSqrt` | square root in, square root out | Permutes marginalised columns first, then a rank revealing Householder QR in place. No inverse anywhere. The default path. |

### 5.8 `AbsOrderMap`

- **Header:** `include/basalt/utils/imu_types.h`
- **Source:** N/A, struct defined in header.
- **Purpose:** The column layout of the linear system. The single source of truth for where every variable sits.
- **Inheritance:** None.

| Field / Method | Type | Role |
|---|---|---|
| `abs_order_map` | `std::map<int64_t, std::pair<int,int>>` | Frame timestamp to column offset and block width. Ordered, so iteration is oldest to newest. |
| `items` | `size_t` | Number of blocks. |
| `total_size` | `size_t` | Total column count, denoted $N$ throughout this document. |
| `print_order()` | | Diagnostic. |

Block widths are `POSE_SIZE`, which is six, for a pose only keyframe and `POSE_VEL_BIAS_SIZE`, which is fifteen, for a navigation state. The construction rule, frame poses first and frame states second, both ascending in time, is what makes the prior a prefix and is asserted at four sites.

### 5.9 `MargLinData<Scalar>`, `ImuLinData<Scalar>` and `MargData`

- **Header:** `include/basalt/utils/imu_types.h`
- **Source:** N/A, structs defined in header.
- **Purpose:** The three payloads passed into and out of the linearisation.

| Type | Field | Role |
|---|---|---|
| `MargLinData` | `is_sqrt` | Fixed for the estimator's lifetime from `vio_sqrt_marg`. Selects the meaning of the next two fields. |
| | `H` | The square root factor $\mathbf{J}_\text{marg}$ when `is_sqrt`, otherwise the information matrix $\mathbf{H}^\ast$. The name is misleading in the default configuration. |
| | `b` | The residual $\mathbf{r}_\text{marg}$ when `is_sqrt`, otherwise the information vector $\mathbf{b}^\ast$. |
| | `order` | The prior's own column layout, always a prefix of the current one. |
| `ImuLinData` | `g` | Gravity in the world frame. |
| | `gyro_bias_sqrt_weight`, `accel_bias_sqrt_weight` | Bias random walk weights, already square rooted. |
| | `imu_meas` | Timestamp to preintegrated measurement pointer. |
| `MargData` | `aom`, `abs_H`, `abs_b` | The full un-eliminated system, pushed to the mapping layer before the elimination runs. |
| | `kfs_all`, `kfs_to_marg` | Which keyframes existed and which are departing. |
| | `use_imu` | True for the inertial estimator, false for the visual one. |

### 5.10 `PoseStateWithLin<Scalar>` and `PoseVelBiasStateWithLin<Scalar>`

- **Header:** `include/basalt/utils/imu_types.h`
- **Source:** N/A, structs defined in header.
- **Purpose:** State wrappers implementing the First-Estimate Jacobians convention of §2.16.
- **Inheritance:** None.

| Field / Method | Role |
|---|---|
| `linearized` | Whether this variable's linearisation point has been frozen. |
| `pose_linearized` / `state_linearized` | The frozen point. Jacobians are evaluated here. |
| `T_w_i_current` / `state_current` | The current estimate. Residuals are evaluated here. |
| `delta` | Accumulated tangent offset between the two. The quantity `computeDelta` gathers. |
| `setLinTrue()` | Freezes the point. Asserts the offset is zero at that instant. |
| `applyInc(inc)` | Moves the linearisation point while unfrozen, accumulates into `delta` once frozen. |
| `getPoseLin()` / `getStateLin()` | The frozen point, for Jacobians. |
| `getPose()` / `getState()` | The current estimate, for residuals. |
| `backup()` / `restore()` | Snapshot and revert, for step rejection. |

The fifteen dimensional tangent ordering is translation, rotation, velocity, gyroscope bias, accelerometer bias, at indices zero to two, three to five, six to eight, nine to eleven and twelve to fourteen respectively.

### 5.11 `LandmarkDatabase<Scalar>`, `Keypoint<Scalar>` and `TimeCamId`

- **Headers:** `include/basalt/vi_estimator/landmark_database.h`, `include/basalt/utils/common_types.h`
- **Source:** `src/vi_estimator/landmark_database.cpp`
- **Purpose:** The landmark store and the observation index. The principal input to the linearisation.

| Type | Field or method | Role |
|---|---|---|
| `TimeCamId` | `frame_id`, `cam_id` | A view, being one camera at one timestamp. A stereo frame contributes two. |
| `Keypoint` | `direction`, `inv_dist` | The three landmark parameters, a bearing and an inverse distance. |
| | `host_kf_id` | The view whose frame the parameters are expressed in. Fixed at creation. |
| | `obs` | Every observing view mapped to its observed pixel. |
| `LandmarkDatabase` | `kpts` | Landmark identifier to `Keypoint`. |
| | `observations` | Host view, then target view, then the set of landmarks that pair observes. The nesting the constructor walks. |
| | `getObservations()` | Const reference to the index, zero copy. |
| | `getLandmarks()`, `getLandmark(id)` | Const reference to the store, and element access that throws if absent. |
| | `removeKeyframes(...)` | Drops landmarks whose host departs, strips observations from departing targets, drops landmarks left with fewer than two observations. |
| | `backup()`, `restore()` | Fan out to every landmark, for step rejection. |

### 5.12 `RelPoseLin<Scalar>`

- **Header:** `include/basalt/linearization/landmark_block.hpp`
- **Source:** N/A, struct defined in header.
- **Purpose:** One entry of the relative pose cache, shared by every landmark observed on the same host and target pair.
- **Inheritance:** None.

| Field | Type | Role |
|---|---|---|
| `T_t_h` | $4\times 4$ | The relative pose, target from host, including the camera extrinsics. |
| `d_rel_d_h` | $6\times 6$ | Derivative of the relative pose tangent with respect to the host body pose tangent. |
| `d_rel_d_t` | $6\times 6$ | The same with respect to the target. Carries the opposite sign. |

### 5.13 `DenseAccumulator<Scalar>` and `BlockDiagonalAccumulator<Scalar>`

- **Headers:** `include/basalt/optimization/accumulator.h`, `include/basalt/linearization/block_diagonal.hpp`
- **Purpose:** Receivers for scattered block contributions.

| Type | Method | Role |
|---|---|---|
| `DenseAccumulator` | `addH<ROWS,COLS>(i, j, data)` | Bounds checked block addition at a given offset, with the block size a compile time constant. |
| | `addB<ROWS>(i, data)` | The same for the vector. |
| | `getH()`, `getB()`, `reset(size)` | Access and initialisation. |
| `BlockDiagonalAccumulator` | `add(idx, data)` | Accumulates a block diagonal preconditioner. Dead in the shipped path. |

### 5.14 Configuration Fields

- **Header:** `include/basalt/utils/vio_config.h`
- **Source:** `src/utils/vio_config.cpp`

| Field | Default | Effect on linearisation |
|---|---|---|
| `vio_linearization_type` | `ABS_QR` | Selects the strategy. |
| `vio_sqrt_marg` | true | Selects the representation of the prior, and therefore which elimination routine runs. |
| `vio_obs_std_dev` | 0.5 | Pixel noise. Divides every visual row. Also rescales the effective Huber threshold. |
| `vio_obs_huber_thresh` | 1.0 | Robustifier threshold, in whitened units. |
| `vio_max_iterations` | 7 | Outer iteration cap. |
| `vio_use_lm` | false | When false the outer loop is Gauss-Newton with diagonal regularisation. |
| `vio_lm_lambda_initial` / `_min` / `_max` | 1e-4, 1e-6, 1e2 | Damping schedule bounds. |
| `vio_scale_jacobian` | true | Enables column scaling. |
| `vio_init_pose_weight` | 1e8 | Initial prior on the four unobservable directions. |
| `vio_init_ba_weight` | 1e1 | Applied to indices nine to eleven, which are the gyroscope bias. See §3.2.2. |
| `vio_init_bg_weight` | 1e2 | Applied to indices twelve to fourteen, which are the accelerometer bias. See §3.2.2. |
| `vio_marg_lost_landmarks` | true | Whether landmarks leaving the window are marginalised or dropped. |

---

## 6. Bibliography

<a id="bib-1"></a>
[1] Demmel, N., Sommer, C., Cremers, D., and Usenko, V. (2021). Square Root Bundle Adjustment for Large-Scale Reconstruction. In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pages 11723 to 11732. arXiv:2103.01843. The primary source for the QR based landmark elimination of §2.11 and §3.1.6, including the equivalence proof with the Schur complement.

<a id="bib-2"></a>
[2] Demmel, N., Schubert, D., Sommer, C., Cremers, D., and Usenko, V. (2021). Square Root Marginalization for Sliding-Window Bundle Adjustment. In Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV), pages 13260 to 13268. arXiv:2109.02182. The sliding window extension, and the source of the square root marginalisation of §3.3.3.

<a id="bib-3"></a>
[3] Usenko, V., Demmel, N., Schubert, D., Stückler, J., and Cremers, D. (2020). Visual-Inertial Mapping with Non-Linear Factor Recovery. IEEE Robotics and Automation Letters, volume 5, number 2, pages 422 to 429. DOI 10.1109/LRA.2019.2961227. arXiv:1904.06504. The paper this codebase implements.

<a id="bib-4"></a>
[4] Mazuran, M., Burgard, W., and Tipaldi, G. D. (2015). Nonlinear Factor Recovery for Long-Term SLAM. The International Journal of Robotics Research.

<a id="bib-5"></a>
[5] Mazuran, M., Tipaldi, G. D., Spinello, L., and Burgard, W. (2014). Nonlinear Graph Sparsification for SLAM. In Robotics, Science and Systems (RSS).

<a id="bib-6"></a>
[6] Forster, C., Carlone, L., Dellaert, F., and Scaramuzza, D. (2017). On-Manifold Preintegration for Real-Time Visual-Inertial Odometry. IEEE Transactions on Robotics, volume 33, number 1, pages 1 to 21. Note that `basalt` does not use this paper's rotation residual convention, as recorded in `context/vio_residuals_and_priors.md`.

<a id="bib-7"></a>
[7] Golub, G. H., and Van Loan, C. F. (2013). Matrix Computations, fourth edition. Johns Hopkins University Press. The reference for QR, Householder and Givens. The source at `landmark_block_abs_dynamic.hpp:444-454` cites its Algorithm 5.2.4 by page number.

<a id="bib-8"></a>
[8] Triggs, B., McLauchlan, P. F., Hartley, R. I., and Fitzgibbon, A. W. (2000). Bundle Adjustment, a Modern Synthesis. In Vision Algorithms, Theory and Practice, Lecture Notes in Computer Science volume 1883, pages 298 to 372. The standard treatment of the arrowhead structure and of robust cost in bundle adjustment.

<a id="bib-9"></a>
[9] Huber, P. J. (1964). Robust Estimation of a Location Parameter. The Annals of Mathematical Statistics, volume 35, number 1, pages 73 to 101. The origin of the robustifier of §2.5.

<a id="bib-10"></a>
[10] Nocedal, J., and Wright, S. J. (2006). Numerical Optimization, second edition. Springer. The reference for the trust region reading of damping in §2.4.

<a id="bib-11"></a>
[11] Björck, Å. (1996). Numerical Methods for Least Squares Problems. SIAM. The reference for the conditioning arguments of §2.13.

<a id="bib-12"></a>
[12] Householder, A. S. (1958). Unitary Triangularization of a Nonsymmetric Matrix. Journal of the ACM, volume 5, number 4, pages 339 to 342.

<a id="bib-13"></a>
[13] Givens, W. (1958). Computation of Plane Unitary Rotations Transforming a General Matrix to Triangular Form. Journal of the Society for Industrial and Applied Mathematics, volume 6, number 1, pages 26 to 50.

<a id="bib-14"></a>
[14] Levenberg, K. (1944). A Method for the Solution of Certain Non-Linear Problems in Least Squares. Quarterly of Applied Mathematics, volume 2, pages 164 to 168.

<a id="bib-15"></a>
[15] Marquardt, D. W. (1963). An Algorithm for Least-Squares Estimation of Nonlinear Parameters. Journal of the Society for Industrial and Applied Mathematics, volume 11, number 2, pages 431 to 441. The source of the diagonal scaling used at `sqrt_keypoint_vio.cpp:1358-1361`.

<a id="bib-16"></a>
[16] Nielsen, H. B. (1999). Damping Parameter in Marquardt's Method. Technical Report IMM-REP-1999-05, Technical University of Denmark. The update rule implemented at `sqrt_keypoint_vio.cpp:1492-1558`.

<a id="bib-17"></a>
[17] Huang, G. P., Mourikis, A. I., and Roumeliotis, S. I. (2010). Observability-Based Rules for Designing Consistent EKF SLAM Estimators. The International Journal of Robotics Research, volume 29, number 5, pages 502 to 528. The observability and consistency argument underlying §2.15 and §2.16.

<a id="bib-18"></a>
[18] Sibley, G., Matthies, L., and Sukhatme, G. (2010). Sliding Window Filter with Application to Planetary Landing. Journal of Field Robotics, volume 27, number 5, pages 587 to 608.

<a id="bib-19"></a>
[19] Leutenegger, S., Lynen, S., Bosse, M., Siegwart, R., and Furgale, P. (2015). Keyframe-Based Visual-Inertial Odometry Using Nonlinear Optimization. The International Journal of Robotics Research, volume 34, number 3, pages 314 to 334.

<a id="bib-20"></a>
[20] Hartley, R., and Zisserman, A. (2004). Multiple View Geometry in Computer Vision, second edition. Cambridge University Press.

<a id="bib-21"></a>
[21] Usenko, V., Demmel, N., and Cremers, D. (2018). The Double Sphere Camera Model. In International Conference on 3D Vision (3DV). DOI 10.1109/3DV.2018.00069. arXiv:1807.08957. One of the camera models the projection of §3.1.4 dispatches to.

<a id="bib-22"></a>
[22] Solà, J., Deray, J., and Atchuthan, D. (2018). A Micro Lie Theory for State Estimation in Robotics. arXiv:1812.01537. A concise treatment of the manifold material of §2.1.

<a id="bib-23"></a>
[23] Barfoot, T. D. (2017). State Estimation for Robotics. Cambridge University Press.

<a id="bib-24"></a>
[24] Zhang, F., editor (2005). The Schur Complement and Its Applications. Springer. The reference for §2.10.

---

## 7. Related Documents

| Document | Relationship |
|---|---|
| `doc/VIO.md` | The estimation mathematics. §2.2.4 and §2.2.5 give the reprojection residual and its Jacobians, §3.1.1 and §3.1.2 the inertial ones, §3.1.3 the marginalisation residual, §3.1.4 the full objective and §2.2.8 the terminal QR equation this document derives. |
| `doc/Marginalisation.md` | The Gauss-Newton derivation in §2, the Schur complement in §3.2, the orchestration in §3.3 and the marginalisation residual derivation in Appendix A, whose §A.1.1 through §A.1.4 supply four preliminaries this document cites rather than repeats. |
| `doc/Mapping.md` | The global mapping problem, which uses the classical Schur complement path rather than this framework. See §3.6.1 and §3.6.2. |
| `doc/LocalMapper.md` | The local mapper, which inherits the mapper's solver unchanged. See §7.7. |
| `context/vio_residuals_and_priors.md` | The authoritative record of the inertial residual conventions and of the initial prior, including the bias weight transposition of §3.2.2. |
