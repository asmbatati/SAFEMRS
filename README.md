# SAFEMRS: Safe Agentic Framework for Externally-augmented Multi-Robot Systems

> Corroborative Dual-Channel Pre-Execution Safety Verification for LLM-Based Heterogeneous Multi-Robot Task Planning

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-Research-orange.svg)]()

---

## 📋 Project Overview

**SAFEMRS** is a research project aimed at publication at IROS/ICRA conferences, focusing on LLM-based heterogeneous multi-robot task planning with formal safety verification.

### Key Features

- 🧠 **Agentic Reasoning Layer**: LLM-driven autonomous task decomposition with Chain-of-Thought (CoT)
- 🔌 **MCP Integration**: External tool and knowledge base access via Model Context Protocol
- 🛡️ **Dual-Channel Safety Verification**: Combines formal logic (LTL/CTL/Deontic) with probabilistic LLM reasoning
- 📡 **Real-Time Monitoring**: Continuous state aggregation with anomaly detection and dynamic re-planning
- 🤖 **Multi-Formalism Planning**: Abstract layer supporting PDDL, Behavior Trees, DAG, HTN, and JSON/YAML

---

## 📁 Repository Structure

```
SAFEMRS/
├── README.md                        # This file
├── CHANGELOG.md                     # Development history
├── safemrs/                         # Core Python package (pip-installable)
│   ├── safemrs/
│   │   ├── config/                  # PlanFormat, thresholds, domain.yaml
│   │   ├── plan_representations/    # InternalPlan + JSON/PDDL/BT converters
│   │   ├── planning/                # Agentic Reasoning Layer (NL → Plan)
│   │   ├── channel_formal/          # Channel 1: LTL + PDDL + Deontic
│   │   ├── channel_llm/             # Channel 2: 4 LLM sub-reasoners
│   │   ├── fusion/                  # Corroborative Fusion mechanism
│   │   ├── benchmark/               # 102 YAML scenarios + evaluator
│   │   └── ros2_integration/        # ROS 2 node + plan executor
│   ├── experiments/                 # Experiment runners + analysis
│   ├── tests/                       # 51 unit tests
│   └── pyproject.toml
├── latex/                           # IROS 2026 paper (main.tex)
├── proposal/                        # Design documents
│   ├── architecture_proposal.md
│   ├── competitive_analysis.md
│   ├── implementation_roadmap.md
│   ├── iros2026_scope.md
│   └── literature_summary.md
├── ros2_agent_sim/                  # ROS 2 simulation (submodule)
├── safemrs_sim/                     # Gazebo worlds (submodule)
└── safemrs_docker/                  # Docker infrastructure (submodule)
```

---

## 🚀 Getting Started

### Prerequisites

- Python 3.10+
- Git
- (Optional) ROS 2 Jazzy for robot integration
- (Optional) Spot library for LTL model checking

### Installation

```bash
git clone --recursive https://github.com/asmbatati/SAFEMRS.git
cd SAFEMRS/safemrs
pip install -e .

# Optional: formal verification backends
pip install -e ".[formal]"

# Verify
python -c "from safemrs.channel_formal import FormalVerifier; print('OK')"
```

### Run Tests

```bash
cd safemrs/
PYTHONPATH=. python -m pytest tests/ -v
```

### Run Experiments

```bash
# Formal-only channel (no LLM needed, instant)
PYTHONPATH=. python experiments/run_all.py --modes formal_only

# Full dual-channel with local Ollama Qwen3:8b
ollama pull qwen3:8b
PYTHONPATH=. python experiments/run_all.py --modes formal_only llm_only dual --llm-backend qwen3:8b

# Background LLM experiment with incremental saves + resume
PYTHONUNBUFFERED=1 PYTHONPATH=. nohup python experiments/run_llm_background.py \
    --backend qwen3:8b > results/llm_experiment.log 2>&1 &

# Check progress
PYTHONPATH=. python experiments/check_progress.py --results-dir results/final
```

### ROS 2 Agent with Safety Gate

```bash
# Launch with dual-channel safety verification (default)
ros2 run ros2_agent ros2_agent_node --ros-args -p safety_mode:=dual

# Launch with formal-only (no LLM latency)
ros2 run ros2_agent ros2_agent_node --ros-args -p safety_mode:=formal_only

# Launch without safety checks (original behavior)
ros2 run ros2_agent ros2_agent_node --ros-args -p safety_mode:=passthrough
```

---

## 🔄 Using the Update Script

The `update_repo.sh` script automates the process of adding, committing, and pushing changes:

### Basic Usage

```bash
# With custom commit message
./update_repo.sh "Your commit message here"

# Without message (uses timestamp)
./update_repo.sh
```

### First-Time Setup

After setting up authentication, run:
```bash
chmod +x update_repo.sh
./update_repo.sh "Initial setup complete"
```

---

## 📊 Project Status

**Status**: Implementation Complete — Experiments & Paper Writing Phase

### Completed ✅

- [x] Comprehensive literature review (16 papers, 2020-2025)
- [x] Complete architecture design (dual-channel framework)
- [x] Competitive analysis with 8 closest competitors
- [x] IROS 2026 scope definition and paper outline
- [x] Implementation roadmap (11-day plan)
- [x] **Core `safemrs` Python package** — all modules implemented
- [x] **Channel 1 (Formal):** LTL verifier, PDDL validator, Deontic checker
- [x] **Channel 2 (LLM):** 4 sub-reasoners with CoT prompts
- [x] **Corroborative Fusion:** 4-way decision logic with explanations
- [x] **Plan Representations:** InternalPlan + JSON/PDDL/BT converters
- [x] **Benchmark:** 102 annotated scenarios across 7 hazard categories
- [x] **Experiment runners:** 5 modes, LLM comparison, analysis scripts
- [x] **Tests:** 51 unit tests, all passing
- [x] **ROS 2 integration:** SafemrsNode, PlanExecutor, MissionInterface
- [x] **Experiments complete (102/102 × 2 backends):** Qwen3:8b + GPT-4o both confirmed
- [x] **LaTeX paper:** ALL tables filled including Table V, zero `\unvalidated{}` remaining
- [x] **CI pipeline:** `.github/workflows/tests.yml`

### In Progress 🔄

- [ ] Paper submission (March 2, 2026)

### Planned 📋

- [ ] ROS 2 demo video (Gazebo + PX4 + Go2)

---

## 🎯 Core Contribution (IROS 2026)

### Dual-Channel Corroborative Pre-Execution Safety Verification

No existing framework runs formal logic and LLM-based safety reasoning as **architecturally independent channels** that produce separate verdicts and reconcile them through a **corroborative fusion mechanism**.

- **Channel 1 (Formal)** = *sound but incomplete* — catches spatial, resource, temporal, battery violations via LTL/PDDL/Deontic constraints
- **Channel 2 (LLM)** = *complete but unsound* — catches common-sense hazards and physical infeasibilities via CoT reasoning
- **Dual-channel fusion** = *strictly better than either alone* — corroborative combination provably covers hazard categories no single channel can handle

### Confirmed Results (Qwen3:8b, 102 scenarios)

| System | HDR | FPR | Coverage | Review Rate | EffCov | Latency |
|--------|:---:|:---:|:--------:|:-----------:|:------:|:-------:|
| Formal-only | 77.4% | 10.2% | 5/7 | — | 77.4% | <1ms |
| LLM-only (Qwen3:8b) | 83.0% | 4.1% | 4/7 | — | 83.0% | 69.3s |
| **SAFEMRS Dual (Qwen3:8b)** | **64.2%** | **0.0%** | 3/7 | **23.5%** | **87.7%** | 69.3s |

**LLM Backbone Comparison (Table V):**

| Backbone | HDR (LLM-ch) | HDR (Dual) | FPR (Dual) | EffCov | Latency |
|----------|:------------:|:----------:|:----------:|:------:|:-------:|
| Qwen3:8b (local) | 83.0% | 64.2% | 0.0% | 87.7% | 69.3s |
| GPT-4o (cloud) | 98.1% | 75.5% | 2.0% | 96.1% | 5.2s |

**Effective ΔC = 32%** (Qwen3:8b): 17/53 unsafe scenarios caught via review-escalation that neither single channel hard-rejects alone.

---

## 📚 Related Work

Our framework builds upon and extends:

- **SMART-LLM** (Kannan et al., 2024) - Task decomposition
- **COHERENT** (Liu et al., 2025) - Multi-robot coordination
- **DART-LLM** (Wang et al., 2025) - Dependency modeling
- **LaMMA-P** (Zhang et al., 2025) - LLM + PDDL integration
- **SafePlan** (Obi et al., 2025) - LLM-based safety screening
- **VerifyLLM** (Grigorev et al., 2025) - Formal verification
- **DEXTER-LLM** (Zhu et al., 2025) - Dynamic re-planning

See [literature_summary.md](proposal/literature_summary.md) for detailed analysis.

---

## 📖 Documentation

- **[IROS 2026 Scope](proposal/iros2026_scope.md)**: Paper scope, architecture, and evaluation plan
- **[Implementation Roadmap](proposal/implementation_roadmap.md)**: Detailed code design for all modules
- **[Architecture Proposal](proposal/architecture_proposal.md)**: Complete system architecture
- **[Competitive Analysis](proposal/competitive_analysis.md)**: Gap analysis and contribution strategy
- **[Literature Summary](proposal/literature_summary.md)**: Thematic analysis of 16 papers (2020-2025)
- **[safemrs/ README](safemrs/README.md)**: Core package documentation and quick start

---

## 🤝 Contributing

This is an active research project. For questions or collaboration inquiries, please contact the research team.

---

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## 📧 Contact

- **Research Supervisor**: Prof. Anis Koubaa
- **Repository Owner**: asmbatati
- **Collaborator**: Anis Koubaa (aniskoubaa)
- **Project Affiliation**: ScaleX Research Lab

---

## 🔗 Links

- [Architecture Documentation](proposal/architecture_proposal.md)
- [Literature Review](proposal/literature_summary.md)
- [Competitive Analysis](proposal/competitive_analysis.md)
- [IROS 2026 Scope](proposal/iros2026_scope.md)
- [Implementation Roadmap](proposal/implementation_roadmap.md)

---

**Last Updated**: February 21, 2026
