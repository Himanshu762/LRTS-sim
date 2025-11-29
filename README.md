# LRTS-sim

> built as a demo for our solution first for Toyota Mobility Foundation Sustainable Cities challenge then the final form for Transport Stack Open Innovation Challenge by FITT IIT Delhi

LRTS-sim is a lightweight simulation environment for modeling and visualizing Local Rickshaw Transit System behavior before full deployment. It pairs Python-based simulation logic with HTML output pages or lightweight templates to communicate zone dynamics, wait times, and fleet distribution.

## Simulation Screenshot (Image 5)

![Simulation Map & Capacity Analysis](screenshot/screenshot.png)

(Ensure the file exists at `screenshot/screenshot.png`. This image demonstrates live zone coverage, trip states, and capacity analysis.)

## Purpose

This repository helps explore:
- How fleet size and distribution affect average passenger wait time
- Zone coverage optimization around metro hubs
- Subscription pass utilization modeling
- Baseline metrics for daily rides, driver activity, and landmark connectivity

## Tech Stack

- Python (simulation logic, data processing)
- HTML (static or templated visualization)
- Procfile (deployment process specification)

## Suggested Structure

```
simulation/
  engine.py
  models/
    zone.py
    fleet.py
    demand.py
  analytics/
    metrics.py
templates/
  index.html
  zones.html
static/
  css/
  js/
screenshots/
  sim-screenshot.png
Procfile
```

## Getting Started

```bash
git clone https://github.com/Himanshu762/LRTS-sim.git
cd LRTS-sim
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python simulation/engine.py  # or flask/fastapi launcher if applicable
```

Open the served HTML (or generated reports) in a browser to inspect simulation results.

## Core Concepts

- Zone: Geographic cluster around a transit/metro node
- Fleet: Set of rickshaws assigned to zones with dynamic rebalancing
- Demand: Modeled passenger requests per time slice
- Pass Model: Subscription tiers altering trip frequency & travel paths

## Extending the Simulation

1. Add a new demand curve: implement in `demand.py`.
2. Adjust rebalancing strategy: modify `fleet.py`.
3. Integrate real data: create adapters under `analytics/`.
4. Export results: add CSV/JSON writers or simple APIs.

## Sample Metrics (Hypothetical)

- Mean Wait Time
- Fleet Utilization %
- Subscription Tier Adoption
- Zone Saturation Ratio

## Deployment

If using a platform like Heroku (Procfile suggests this):
```bash
heroku create
git push heroku main
heroku open
```
(Adjust for current hosting providers if different.)

## Contributing

- Document new parameters in README or a `docs/` folder.
- Provide reproducible seeds for stochastic simulations.
- Include unit tests for deterministic functions (e.g., cost calculators).

## License

Add a LICENSE file (MIT / Apache-2.0 / etc.) to clarify usage.

## Acknowledgements

- Toyota Mobility Foundation Sustainable Cities Challenge
- FITT IIT Delhi Transport Stack Open Innovation Challenge
- Python & open data communities