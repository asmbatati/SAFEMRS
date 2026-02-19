`# SAFEMRS IROS 2026: Dual-Channel Pre-Execution Safety Verification

> **Target Venue:** IROS 2026 — Pittsburgh, PA | Sep 27 – Oct 1, 2026
> **Paper Deadline:** March 2, 2026
> **Scope Decision:** Dual-Channel (Formal Logic + LLM-Based) Pre-Execution Safety Verification

---

## 1. Scope Decision & Justification

### 1.1 Chosen Contribution: Dual-Channel Corroborative Pre-Execution Safety Verification

We scope the IROS 2026 paper to a **dual-channel pre-execution safety verification** framework for LLM-based heterogeneous multi-robot task planning. Runtime enforcement (CBF) and real-time monitoring (RTM) are deferred to future work (ICRA 2027 / RA-L).

### 1.2 Why Dual-Channel Is Sufficient Novelty

| Existing Work      | What It Does                                 | What It Misses                                                             |
| ------------------ | -------------------------------------------- | -------------------------------------------------------------------------- |
| **SafePlan**       | LLM CoT safety reasoning only                | No formal logic — safety verdict is itself an LLM output (can hallucinate) |
| **VerifyLLM**      | Formal LTL verification only                 | No common-sense reasoning — misses hazards not expressible in LTL          |
| **LTLCodeGen**     | Syntax-guaranteed LTL generation             | No semantic safety reasoning                                               |
| **NL2HLTL2PLAN**   | Hierarchical formal specifications           | No LLM safety channel                                                      |
| **SAFER**          | CBF runtime enforcement                      | No pre-execution formal verification                                       |
| **S-ATLAS**        | Conformal prediction bounds                  | No formal logic or LLM safety                                              |
| **SAFEMRS (Dual)** | **Formal + LLM corroborative pre-execution** | **Novel combination with fusion mechanism** ✅                             |

> [!IMPORTANT]
> **No existing paper combines formal logic verification AND LLM-based safety reasoning into a corroborative pre-execution pipeline for heterogeneous multi-robot task planning.** The dual-channel fusion with disagreement handling is a genuine, provable, non-incremental contribution.

### 1.3 What We Gain by Focusing on Dual-Channel

| Dropped Component                | Saved Effort | Deferred To         |
| -------------------------------- | ------------ | ------------------- |
| CBF runtime enforcement          | ~4 weeks     | ICRA 2027 (Paper 2) |
| Real-Time Monitoring (RTM)       | ~2 weeks     | ICRA 2027 (Paper 2) |
| Conformal prediction calibration | ~2 weeks     | ICRA 2027 (Paper 2) |
| 8 planning backends              | ~6 weeks     | RA-L (Paper 3)      |
| MCP external tools               | ~3 weeks     | RA-L (Paper 3)      |
| HAL (6 adapters)                 | ~4 weeks     | RA-L (Paper 3)      |

**Result:** Focus entirely on **one deep contribution** implementable in ~11 days.

---

## 2. Paper Positioning

### 2.1 Proposed Title

> _"Corroborative Dual-Channel Pre-Execution Safety Verification for LLM-Based Heterogeneous Multi-Robot Task Planning"_

### 2.2 Core Research Question

> _Does corroborative dual-channel safety verification — combining formal logic (LTL) and LLM-based reasoning — provide strictly better hazard detection than either channel alone for LLM-generated multi-robot task plans?_

### 2.3 Primary Novelty Claim

The two channels are **complementary by design**:

- **Formal channel** = _sound but incomplete_ — catches violations of explicitly specified LTL/PDDL constraints (spatial, temporal, resource), but cannot reason about unspecified common-sense hazards
- **LLM channel** = _complete but unsound_ — can reason about any hazard including common-sense and physical infeasibilities, but may hallucinate safety verdicts
- **Dual-channel fusion** = _strictly better than either alone_ — the corroborative combination provably covers hazard categories that no single channel can handle

---

## 3. Architecture

```
Human Command (Natural Language)
        │
        ▼
┌───────────────────────────────┐
│    Agentic Reasoning Layer    │  ◄── LLM (GPT-4 / Qwen3)
│  - CoT Task Decomposition     │
│  - PDDL Plan Generation       │
│  - DAG Dependency Graph       │
└──────────────┬────────────────┘
               │  Candidate Plan
               ▼
┌──────────────────────────────────────────────────────┐
│         DUAL-CHANNEL PRE-EXECUTION SAFETY            │
│                                                      │
│  ┌────────────────────┐   ┌───────────────────────┐  │
│  │    Channel 1:      │   │      Channel 2:       │  │
│  │  Formal Logic      │   │   LLM Safety CoT      │  │
│  │                    │   │                       │  │
│  │  - NL → LTL spec   │   │  - Invariant Reasoner │  │
│  │  - PDDL validator  │   │  - Conflict Detector  │  │
│  │  - Model checking  │   │  - Common-sense check │  │
│  │  - Deontic logic   │   │  - Physical feasibil. │  │
│  └─────────┬──────────┘   └──────────┬────────────┘  │
│            │                         │               │
│            ▼                         ▼               │
│  ┌──────────────────────────────────────────────┐    │
│  │         Corroborative Fusion                 │    │
│  │                                              │    │
│  │  Both SAFE    → ✅ Approve for execution     │    │
│  │  Both UNSAFE  → ❌ Reject with explanation   │    │
│  │  Disagree     → ⚠️  Human review / refine    │    │
│  └──────────────────┬───────────────────────────┘    │
└─────────────────────┼────────────────────────────────┘
                      │  Verified Plan
                      ▼
┌─────────────────────────────────────────┐
│         ROS2 Execution Layer            │
│  - PX4 UAV (MAVROS + Gazebo Harmonic)  │
│  - Unitree Go2 (CHAMP controller)       │
│  - Gazebo Harmonic simulation           │
└─────────────────────────────────────────┘
```

---

## 4. Use Case: UAV + UGV Search & Inspection Mission (ROS2)

### 4.1 Scenario Description

A heterogeneous team consisting of:

- **UAV (PX4 drone)** — aerial surveillance, roof inspection
- **UGV (Unitree Go2 quadruped)** — ground-level inspection

**Mission command:**

> _"Inspect the damaged building. The drone should survey the roof and check for structural damage while the quadruped inspects the ground floor entrance. Generate a joint damage report."_

This scenario is:

- ✅ Supported by the existing `ros2_agent_sim` + `ros2_agent_sim_docker` infrastructure
- ✅ Naturally generates diverse safety-critical planning challenges across 7 hazard categories
- ✅ Realistic for IROS reviewers (search & rescue, infrastructure inspection)

### 4.2 Hazard Category Benchmark (50–100 Scenarios)

| #   | Category                         | Example Scenario                                                  | Channel That Catches It                                      |
| --- | -------------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------------ |
| 1   | **Spatial conflicts**            | Drone and Go2 assigned to the same narrow corridor simultaneously | ✅ Formal: `G(¬(loc_drone = corridor ∧ loc_go2 = corridor))` |
| 2   | **Resource conflicts**           | Both robots need the same charging station at the same time       | ✅ Formal: mutex constraint in PDDL                          |
| 3   | **Temporal ordering violations** | Go2 enters building before drone confirms roof stability          | ✅ Formal: `¬enter_building U roof_cleared`                  |
| 4   | **Common-sense hazards**         | Drone assigned to fly indoors in a room with ceiling fans         | ✅ LLM: knows fans are dangerous for rotors                  |
| 5   | **Physical infeasibility**       | Go2 assigned to climb a vertical ladder                           | ✅ LLM: knows quadrupeds cannot climb ladders                |
| 6   | **Battery/range violations**     | Mission plan exceeds UAV maximum flight time                      | ✅ Both: formal numerical constraint + LLM CoT               |
| 7   | **Ordering/dependency errors**   | Damage report generated before inspection is completed            | ✅ Both: LTL + LLM pre/post-conditions                       |

### 4.3 The Killer Empirical Result

The paper must show that **neither channel alone is sufficient** and the dual combination achieves qualitatively better coverage:

| System                        | Spatial | Resource | Temporal | Common-Sense | Physical | Battery | Ordering | **Overall** |
| ----------------------------- | :-----: | :------: | :------: | :----------: | :------: | :-----: | :------: | :---------: |
| Formal-only (VerifyLLM-style) |   ✅    |    ✅    |    ✅    |      ❌      |    ❌    |   ✅    |    ✅    |    ~71%     |
| LLM-only (SafePlan-style)     |   ❌    |  ❌/✅   |    ✅    |      ✅      |    ✅    |   ✅    |    ✅    |    ~71%     |
| **Dual-channel (SAFEMRS)**    |   ✅    |    ✅    |    ✅    |      ✅      |    ✅    |   ✅    |    ✅    |  **95%+**   |

> The key finding: **the two channels are complementary** — formal catches what LLM misses (spatial/resource), LLM catches what formal cannot express (common-sense/physical). Only the combination achieves near-complete coverage.

---

## 5. Implementation Plan (11 Days to March 2 Deadline)

### 5.1 Task Breakdown

| Day     | Task                                                                                                                           | Deliverable                     |
| ------- | ------------------------------------------------------------------------------------------------------------------------------ | ------------------------------- |
| **1–2** | Define PDDL domain for UAV+UGV inspection (actions, predicates, safety axioms) + LTL spec templates                            | `domain.pddl`, LTL spec library |
| **3–4** | Implement **Channel 1** (Formal): NL→LTL translation + PDDL validator + model checker                                          | `formal_verifier.py`            |
| **5–6** | Implement **Channel 2** (LLM): CoT safety prompt pipeline — invariant reasoner, conflict detector, common-sense checker        | `llm_safety_reasoner.py`        |
| **7**   | Implement **Corroborative Fusion**: verdict combination logic + disagreement handler + explanation generator                   | `fusion.py`                     |
| **8–9** | Create scenario dataset (50–100 labeled safe/unsafe plans across 7 hazard categories) + ROS2 integration with `ros2_agent_sim` | Benchmark + launch files        |
| **10**  | Run experiments: single-channel ablation vs. dual-channel on all 7 categories                                                  | Results tables + figures        |
| **11**  | Write 6-page paper following IROS template                                                                                     | `main.tex` ready to submit      |

### 5.2 Technical Stack

| Component            | Technology                                                                 |
| -------------------- | -------------------------------------------------------------------------- |
| LLM backbone         | Qwen3:8b (local via Ollama) + GPT-4 (cloud, for comparison)                |
| Formal verification  | `unified-planning` (Python) + PDDL validator + `spot` (LTL model checking) |
| LLM safety reasoning | LangChain + custom CoT safety prompt templates                             |
| Simulation           | Gazebo Harmonic + ROS2 Jazzy                                               |
| UAV                  | PX4 SITL + MAVROS                                                          |
| UGV                  | Unitree Go2 via CHAMP controller                                           |
| Benchmark            | Custom 50–100 scenario dataset + RoCoBench subset (existing)               |

---

## 6. Evaluation Metrics

| Metric                          | Definition                                                            | Target                                  |
| ------------------------------- | --------------------------------------------------------------------- | --------------------------------------- |
| **Hazard Detection Rate (HDR)** | % of unsafe plans correctly flagged as unsafe                         | ≥ 95% (dual), show single-channel < 75% |
| **False Positive Rate (FPR)**   | % of safe plans incorrectly flagged as unsafe                         | ≤ 10%                                   |
| **Safety Coverage**             | % of the 7 hazard categories with HDR > 80%                           | 7/7 (dual), 4-5/7 (single-channel)      |
| **Channel Complementarity**     | % of hazards caught by dual that neither single channel catches alone | Target > 20%                            |
| **Disagreement Rate**           | How often channels disagree (reveals ambiguous/"hard" cases)          | Analyze and categorize                  |
| **Explanation Quality**         | Human evaluation of rejection explanations (1–5 scale)                | ≥ 4/5                                   |
| **Latency**                     | End-to-end verification time before plan execution                    | ≤ 5 seconds per plan                    |

---

## 7. Paper Structure (6 Pages, IROS Template)

```
I.   INTRODUCTION (1 page)
     - Problem: LLMs plan well but unsafely; formal verification is incomplete;
       LLM-only safety hallucinates
     - Gap: No work combines formal logic + LLM safety reasoning
       corroboratively for multi-robot planning
     - Contribution bullets:
         (1) Dual-channel corroborative safety framework
         (2) Formal proof of channel complementarity
         (3) 7-category heterogeneous MRS safety benchmark

II.  RELATED WORK (0.75 pages)
     - LLM-based multi-robot planning (SMART-LLM, COHERENT, DART-LLM, LaMMA-P)
     - Safety in LLM planning (SafePlan, VerifyLLM, LTLCodeGen, NL2HLTL2PLAN)
     - Clear gap statement: none combines both channels corroboratively

III. PROBLEM FORMULATION (0.75 pages)
     - Formal definition of dual-channel verification problem
     - Definitions: safety coverage, soundness, completeness
     - Theorem: formal channel is sound-but-incomplete,
                LLM channel is complete-but-unsound,
                dual fusion is strictly better on coverage-soundness frontier

IV.  SAFEMRS DUAL-CHANNEL ARCHITECTURE (1.5 pages)
     - System overview figure (one main figure)
     - Channel 1: Formal Logic Verifier (LTL + PDDL + Deontic)
     - Channel 2: LLM Safety CoT Reasoner (invariants, conflicts,
                  common-sense, physical feasibility)
     - Corroborative Fusion mechanism
     - Integration with ROS2/Gazebo

V.   EXPERIMENTS (1.5 pages)
     - Benchmark: 7 hazard categories, 50-100 scenarios, UAV+UGV inspection
     - Baselines: SafePlan (LLM-only), VerifyLLM (formal-only),
                  COHERENT (no safety), LaMMA-P (PDDL-only validation)
     - Results table: per-category HDR across all systems
     - Ablation: formal-only vs. LLM-only vs. dual (main result)
     - Disagreement analysis: what do the channels disagree on?
     - Latency analysis

VI.  CONCLUSION (0.5 pages)
     - Key finding: channels are complementary; dual coverage is strictly better
     - Limitations: pre-execution only (CBF runtime → future work)
     - Future: triple-channel with CBF enforcement (ICRA 2027)
```

---

## 8. What NOT to Do (Reviewer Rejection Avoidance)

| Mistake                                   | Risk                      | How We Avoid It                                                               |
| ----------------------------------------- | ------------------------- | ----------------------------------------------------------------------------- |
| "We combine SafePlan + VerifyLLM" framing | Sounds incremental        | Frame as **corroborative fusion theory** with formal proof of complementarity |
| Simulation only, no real robots           | Distrust of results       | At least one real-world or high-fidelity sim demo with actual ROS2 execution  |
| Only GPT-4 tested                         | Model-dependent criticism | Test on Qwen3 (local) + GPT-4 (cloud) to show architecture-level contribution |
| No baselines                              | Unquantified significance | Compare against SafePlan, VerifyLLM, COHERENT, LaMMA-P                        |
| Vague safety claims                       | No quantification         | All claims backed by HDR/FPR/coverage metrics                                 |
| No failure analysis                       | Lack of honesty           | Include disagreement analysis and per-category failure modes                  |

---

## 9. Multi-Paper Strategy (No Salami-Slicing)

| Paper              | Venue         | Core Focus                                                   | Status            |
| ------------------ | ------------- | ------------------------------------------------------------ | ----------------- |
| **Paper 1 (this)** | **IROS 2026** | Dual-channel corroborative pre-execution safety verification | 🔴 Draft by Mar 2 |
| Paper 2            | ICRA 2027     | Triple-channel + CBF runtime + Receding Horizon Planning     | 🔵 Start May 2026 |
| Paper 3            | RA-L          | Agentic cognitive proxy + MCP + Robot Resumes + HAL          | 🔵 Start Jul 2026 |

Each paper has a **different research question, different baselines, different evaluation metrics** — no salami-slicing.

---

## 10. Suggested Paper Title (Final)

> **"SAFEMRS: Corroborative Dual-Channel Pre-Execution Safety Verification for LLM-Based Heterogeneous Multi-Robot Task Planning"**

This title signals:

1. The new framework name (SAFEMRS)
2. A new multi-channel verification approach (corroborative)
3. That it is pre-execution (honest scope)
4. Target domain (LLM-based multi-robot planning)
