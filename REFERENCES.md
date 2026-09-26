# Reference basis for the model

The implementation is not copied from one system. It combines standard railway longitudinal dynamics, observer-based adhesion handling, and rail-map constrained localization.

## Railway dynamics / textbooks

1. Iwnicki, S.; Spiryagin, M.; Cole, C.; McSweeney, T. (eds.). **Handbook of Railway Vehicle Dynamics, 2nd ed.** CRC Press, 2019. Relevant topics: powered rail vehicles, wheel–rail contact, tribology, longitudinal train dynamics, simulation and field testing.
2. Garg, V. K.; Dukkipati, R. V. **Dynamics of Railway Vehicle Systems.** Academic Press/Elsevier. Relevant topics: wheel–rail rolling contact, railway vehicle models, train dynamics, validation of models.
3. Davis, W. J. **The Tractive Resistance of Electric Locomotives and Cars** (1926). Classical empirical basis for `A + B v + C v^2` running resistance.

## Adhesion / slip estimation

4. O. Polach. **Creep forces in simulations of traction vehicles running on adhesion limit.** Wear 258 (2005), 992–1000. Railway-specific nonlinear creep/adhesion modeling.
5. Ward et al. / Vehicle System Dynamics literature on **multiple-model estimation of wheel–rail contact conditions and adhesion**: banks of Kalman filters and residual-based mode identification.
6. Recent adaptive nonlinear Kalman-filter adhesion-control literature uses model-based train-speed estimation to maintain robust operation near adhesion limits.

## Rail/tram localization practice

7. Korea Railroad Research Institute (2025), **Development of a Localization System for a Self-Driving Tram Vehicle**: GNSS/INS + tachometer + tag + digital map; demonstrates the importance of track-constrained localization.
8. **A Digital Track Map-Assisted SINS/OD Fusion Algorithm for Onboard Train Localization** (Applied Sciences, 2024): odometer + digital track map map-matching reduces accumulated position error.
9. **RailLoMer** and later multimodal rail-odometry work: odometer is treated as a primary motion source and rail geometry is explicitly used as a state-estimation constraint.
10. Siemens **Autonomous Tram / AStriD** research: digital maps are a core component of depot localization; autonomous tram prototypes use redundant sensing and map context.

The hackathon sensor restrictions are much tighter than these real systems. Therefore we transfer the mathematical architecture (state estimator + map constraint + fault handling), not their forbidden sensors.
