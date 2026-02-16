# Competitive Analysis & Strategy for a Substantial Contribution to IROS/ICRA

> How does SAFEMRS compare to the state-of-the-art, and what must change to make it a **non-incremental**, high-impact contribution?

---

## 1. Honest Assessment: What Reviewers Will See

Before proposing solutions, we must confront the risk that IROS/ICRA reviewers will perceive the current SAFEMRS proposal as **incremental integration** — combining pieces from existing works without a sufficiently deep standalone contribution.

### The "Integration Trap"

| SAFEMRS Component                      | Reviewer Will Say: "This Already Exists in…"            |
| -------------------------------------- | ------------------------------------------------------- |
| Agentic LLM task decomposition         | SMART-LLM, COHERENT, DART-LLM, LaMMA-P, CLGA           |
| DAG-based dependency modeling          | DART-LLM, LiP-LLM                                      |
| Formal logic verification (LTL)        | VerifyLLM, LTLCodeGen, NL2HLTL2PLAN                     |
| LLM-based safety reasoning (CoT)       | SafePlan                                                 |
| CBF runtime safety enforcement         | SAFER                                                    |
| Probabilistic safety bounds            | S-ATLAS (conformal prediction)                           |
| Dynamic re-planning                    | DEXTER-LLM, LLM-CBT                                     |
| Heterogeneous robot coordination       | COHERENT, AutoHMA-LLM                                   |
| Behavior tree execution                | PLANTOR, LAN2CB, LLM-CBT, Yuan et al., Hoffmeister      |
| PDDL integration                       | LaMMA-P, GMATP-LLM                                      |
| Code generation for robot control      | Code as Policies, ProgPrompt, Code-as-Symbolic-Planner  |
| Multi-agent LLM dialog / negotiation   | RoCo, FCRF, Reasoner (ToM)                               |
| Hybrid LLM + RL coordination           | ICCO, Chen MAS+RL                                        |

> [!CAUTION]
> **If the paper reads as "we combine SafePlan + VerifyLLM + SAFER + DEXTER-LLM + MCP into one system," it will be desk-rejected at top venues.** The expanded literature (46 papers) means the bar is even higher — individual safety mechanisms (CBFs, conformal prediction, formal logic, LLM reasoning) now all exist separately. SAFEMRS must demonstrate that their **integration produces emergent safety properties** impossible with any subset.

---

## 2. Paper-by-Paper Competitive Comparison

### 2.1 vs. SafePlan (Obi et al., 2025) — _Closest competitor on safety_

| Dimension               | SafePlan                                                                                                               | SAFEMRS                                               |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| **Safety mechanism**    | LLM-based CoT with 3-layer screening (societal → organizational → individual) + invariant/pre/post-condition reasoners | Triple-channel: formal logic (LTL/CTL) + LLM CoT + CBF runtime enforcement |
| **Formal verification** | ❌ No formal logic — purely LLM reasoning                                                                              | ✅ LTL/CTL model checking + deontic logic + syntax-guaranteed LTL (LTLCodeGen) |
| **Runtime enforcement** | ❌ Pre-execution screening only                                                                                        | ✅ CBF-based runtime enforcement (inspired by SAFER) |
| **Probabilistic bounds**| ❌ No calibration                                                                                                       | ✅ Conformal prediction calibration (inspired by S-ATLAS) |
| **When it verifies**    | Pre-execution (prompt screening only)                                                                                  | Pre-execution + runtime monitoring + CBF enforcement |
| **Multi-robot**         | Single-robot tasks in AI2-THOR                                                                                         | Heterogeneous multi-robot teams                       |
| **Hallucination risk**  | High — safety verdict is itself an LLM output                                                                          | Lower — formal channel + CBFs provide ground truth   |
| **Conflict detection**  | Implicit in CoT reasoning                                                                                              | Explicit resource/spatial/temporal conflict detection |
| **Benchmark**           | 621 expert-curated prompts                                                                                             | TBD (RoCoBench, LEMMA, MRSC planned)                  |

**Gap SAFEMRS fills**: SafePlan's fundamental weakness is that its safety verdict is **another LLM output** — it can hallucinate safety just as it can hallucinate plans. SAFEMRS addresses this with three independent channels: formal logic provides **verifiable guarantees**, CBFs provide **runtime enforcement**, and conformal prediction provides **calibrated confidence bounds**.

**But is that enough?** Combining existing ideas is still integration risk. The key differentiator is that SAFEMRS's triple-channel fusion produces **emergent safety properties** impossible with any single channel (see §3).

---

### 2.2 vs. VerifyLLM (Grigorev et al., 2025) — _Closest competitor on formal verification_

| Dimension                | VerifyLLM                                            | SAFEMRS                                                                            |
| ------------------------ | ---------------------------------------------------- | ---------------------------------------------------------------------------------- |
### 2.2 vs. VerifyLLM / LTLCodeGen / NL2HLTL2PLAN — _Closest competitors on formal verification_

| Dimension                | VerifyLLM                                             | LTLCodeGen                    | NL2HLTL2PLAN                              | SAFEMRS                                                                            |
| ------------------------ | ----------------------------------------------------- | ----------------------------- | ----------------------------------------- | ---------------------------------------------------------------------------------- |
| **Logic formalism**      | LTL only                                              | LTL (syntax-guaranteed)       | Hierarchical LTL                          | LTL + CTL + Deontic + STL                                                          |
| **Verification method**  | NL → LTL + LLM sliding window                         | Code generation with syntax guarantees | Hierarchical decomposition        | Model checking + LLM CoT + CBF enforcement                                        |
| **Scope**                | Post-planning, pre-execution                          | Pre-execution                 | Pre-execution                             | Pre-execution + runtime                                                            |
| **Multi-robot**          | Single robot                                          | Single robot                  | Multi-robot (scalable)                    | Multi-robot with inter-robot dependencies                                          |
| **Error types detected** | Position errors, missing prerequisites, redundancies  | Syntax errors in LTL          | Specification completeness                | + resource/spatial/temporal conflicts, common-sense hazards, CBF violations         |
| **LLM role**             | Verifier only                                         | Code generator                | Spec generator                            | Planner + verifier + re-planner                                                    |

**Gap SAFEMRS fills**: VerifyLLM verifies single-robot plans; LTLCodeGen guarantees syntactic correctness; NL2HLTL2PLAN scales formal specs via hierarchy. None provides **runtime enforcement**, **multi-paradigm safety integration**, or **agentic reasoning**. SAFEMRS combines their strengths (syntax guarantees from LTLCodeGen, hierarchical specs from NL2HLTL2PLAN, LTL verification from VerifyLLM) with CBF runtime enforcement and LLM safety reasoning.

---

### 2.3 vs. COHERENT (Liu et al., 2025) — _Closest competitor on performance_

| Dimension              | COHERENT                       | SAFEMRS                                     |
| ---------------------- | ------------------------------ | ------------------------------------------- |
| **Planning**           | Centralized LLM with PEFA loop | Agentic LLM with MCP augmentation           |
| **Success rate**       | 97.5%                          | TBD                                         |
| **Safety**             | ❌ None                        | ✅ Triple-channel (formal + LLM + CBF)      |
| **External knowledge** | ❌ Closed-loop                 | ✅ MCP tools + external agents              |
| **Re-planning**        | PEFA feedback loop (internal)  | RTM-triggered (external observation-driven) |
| **Environment**        | Fully observable simulation    | Partially observable                        |

**Gap SAFEMRS fills**: COHERENT achieves impressive results but in **safe, fully observable** environments. It has no safety layer — a dangerous plan will execute if the LLM generates it. It also cannot access external information to ground its reasoning. Furthermore, the recent works RoCo and FCRF show that LLM-based multi-robot dialog and reflection can improve planning quality — SAFEMRS can integrate these collaboration patterns while adding the critical safety layer that all of them lack.

---

### 2.4 vs. DEXTER-LLM (Zhu et al., 2025) — _Closest competitor on dynamic adaptation_

| Dimension                    | DEXTER-LLM             | SAFEMRS                                  |
| ---------------------------- | ---------------------- | ---------------------------------------- |
| **Online re-planning**       | ✅ Yes                 | ✅ Yes                                   |
| **Unknown environments**     | ✅ Yes                 | ✅ Yes (partially observable)            |
| **Safety verification**      | ❌ None                | ✅ Triple-channel (formal + LLM + CBF)   |
| **External resource access** | ❌ No                  | ✅ MCP                                   |
| **Explainability**           | ✅ Verbal explanations | Partial (through CoT traces)             |
| **Optimization**             | Branch-and-bound       | Abstract (LP, MILP, or heuristic search) |

**Gap SAFEMRS fills**: DEXTER-LLM re-plans dynamically but never verifies whether the new plan is safe. LLM-CBT similarly supports closed-loop BT re-planning for UAV-UGV swarms but without formal safety verification. In high-stakes environments (warehouses, hospitals), unverified re-planning is unacceptable. SAFEMRS ensures every re-planned mission passes triple-channel verification before execution.

---

### 2.5 vs. LaMMA-P (Zhang et al., 2025) — _Closest competitor on LLM + formal planning_

| Dimension           | LaMMA-P                  | SAFEMRS                             |
| ------------------- | ------------------------ | ----------------------------------- |
| **Planner**         | PDDL + Fast Downward     | Abstract (8 backends: PDDL, BT, DAG, HTN, STL, FSM, Code, YAML) |
| **Validation**      | PDDL syntactic validator | Triple-channel safety (formal + LLM + CBF runtime) |
| **Task allocation** | LLM-based                | LLM-based + formal optimization     |
| **Benchmark**       | MAT-THOR (70 tasks)      | TBD (MAT-THOR + RoCoBench + LEMMA)  |
| **SR improvement**  | 105% over SMART-LLM      | TBD                                 |

**Gap SAFEMRS fills**: LaMMA-P validates PDDL syntax but not semantic safety. It cannot detect a syntactically valid plan that is semantically dangerous. SAFEMRS supports 8 planning formalisms (not just PDDL) and verifies semantic safety through triple-channel verification including CBF runtime enforcement.

---

### 2.6 vs. LiP-LLM (Obata et al., 2025) — _Closest competitor on optimization_

| Dimension               | LiP-LLM                      | SAFEMRS                            |
| ----------------------- | ---------------------------- | ---------------------------------- |
| **Dependency modeling** | DAG (LLM-generated)          | DAG (within abstract layer)        |
| **Task allocation**     | Linear Programming (optimal) | Configurable (LP, MILP, heuristic) |
| **Safety**              | ❌ None                      | ✅ Triple-channel                  |
| **External knowledge**  | ❌ No                        | ✅ MCP                             |
| **Re-planning**         | ❌ Static                    | ✅ Dynamic (safety-preserving)     |

---

### 2.7 vs. DART-LLM (Wang et al., 2025)

| Dimension        | DART-LLM                   | SAFEMRS                         |
| ---------------- | -------------------------- | ------------------------------- |
| **Dependencies** | DAG-based, QA LLM          | DAG-based within abstract layer |
| **Verification** | VLM-based object detection | Formal logic + LLM CoT          |
| **Execution**    | Open-loop                  | Closed-loop with RTM            |
| **Safety**       | ❌                         | ✅                              |

---

### 2.8 vs. SAFER (Tazarv et al., 2025) — _Closest competitor on runtime safety_

| Dimension               | SAFER                                                   | SAFEMRS                                                    |
| ----------------------- | ------------------------------------------------------- | ---------------------------------------------------------- |
| **Safety mechanism**    | LLM + CBF runtime enforcement                           | Triple-channel: formal logic + LLM CoT + CBF enforcement  |
| **Formal verification** | ❌ No formal logic — relies on CBFs alone               | ✅ LTL/CTL model checking + deontic logic                  |
| **Scope**               | Planning + runtime enforcement                          | Pre-execution + runtime + continuous monitoring            |
| **Multi-robot**         | Multi-robot (with safety constraints)                   | Heterogeneous multi-robot with inter-robot dependencies    |
| **Planning formalism**  | Single (LLM-generated)                                  | 8 backends (PDDL, BT, DAG, HTN, STL, FSM, Code, YAML)     |
| **External knowledge**  | ❌ No                                                   | ✅ MCP tools + external agents                             |

**Gap SAFEMRS fills**: SAFER pioneers LLM+CBF integration but cannot verify **high-level plan correctness** — CBFs enforce low-level safety constraints (e.g., collision avoidance) but cannot catch logical errors (e.g., incorrect task ordering, resource conflicts). SAFEMRS adds formal logic verification and LLM safety reasoning as complementary channels.

---

### 2.9 vs. S-ATLAS (Nayak et al., 2025) — _Closest competitor on probabilistic safety_

| Dimension               | S-ATLAS                                                 | SAFEMRS                                                    |
| ----------------------- | ------------------------------------------------------- | ---------------------------------------------------------- |
| **Safety mechanism**    | Conformal prediction for probabilistic correctness      | Triple-channel (formal + LLM + CBF) with conformal calibration |
| **Guarantees**          | Statistical bounds (user-specified failure rate)         | Statistical bounds + formal guarantees + runtime enforcement |
| **Multi-robot**         | ✅ Multi-robot task allocation                          | ✅ Heterogeneous multi-robot                               |
| **Runtime enforcement** | ❌ Pre-execution only                                   | ✅ CBF-based runtime enforcement                           |
| **Planning formalism**  | Single (LLM-generated)                                  | 8 backends                                                 |

**Gap SAFEMRS fills**: S-ATLAS provides statistical safety guarantees but does not enforce them at runtime. SAFEMRS integrates conformal prediction as one component within a broader safety pipeline that also includes formal verification and CBF enforcement.

---

### 2.10 vs. RoCo (Mandi et al., 2024) — _Closest competitor on multi-agent coordination_

| Dimension               | RoCo                                        | SAFEMRS                                                    |
| ----------------------- | ------------------------------------------- | ---------------------------------------------------------- |
| **Coordination**        | Inter-robot LLM dialog + negotiation        | Agentic reasoning + MCP + multi-agent dialog (adoptable)   |
| **Safety**              | ❌ None                                     | ✅ Triple-channel                                          |
| **Planning formalism**  | Single (dialog-generated plans)             | 8 backends                                                 |
| **Benchmark**           | RoCoBench (6 tasks, 2 robots)               | TBD (including RoCoBench)                                  |

**Gap SAFEMRS fills**: RoCo's robots negotiate plans but have **no safety verification** — a negotiated plan could be dangerous. SAFEMRS can incorporate RoCo-style dialog within the ARL while ensuring all results pass triple-channel verification.

---

### 2.11 vs. LLM-CBT (Nie et al., 2025) — _Closest competitor on closed-loop BTs_

| Dimension               | LLM-CBT                                    | SAFEMRS                                                    |
| ----------------------- | ------------------------------------------- | ---------------------------------------------------------- |
| **Planning output**     | Behavior trees (closed-loop)                | 8 planning formalisms (BT is one option)                   |
| **Re-planning**         | ✅ Dynamic BT re-planning                  | ✅ Safety-preserving re-planning (any formalism)           |
| **Safety**              | ❌ No formal verification                  | ✅ Triple-channel                                          |
| **Multi-robot**         | UAV-UGV swarms                              | Heterogeneous multi-robot (any type)                       |
| **External knowledge**  | ❌ No                                       | ✅ MCP                                                     |

**Gap SAFEMRS fills**: LLM-CBT provides excellent closed-loop BT execution but is locked to a single formalism and has no safety verification. SAFEMRS provides BTs as one of 8 planning backends, all verified by the triple-channel SRL.

---

### 2.12 vs. Code as Policies / ProgPrompt (Liang et al., 2023; Singh et al., 2023) — _Closest competitors on code generation_

| Dimension               | Code as Policies / ProgPrompt               | SAFEMRS                                                    |
| ----------------------- | ------------------------------------------- | ---------------------------------------------------------- |
| **Planning output**     | Python code (executable policies)           | Code generation as one of 8 planning backends              |
| **Safety**              | ❌ Basic assertions only                    | ✅ Triple-channel verification of generated code           |
| **Multi-robot**         | Single robot                                | Heterogeneous multi-robot                                  |
| **External knowledge**  | ❌ API library only                         | ✅ MCP tools + external agents                             |
| **Re-planning**         | ❌ Static                                   | ✅ Dynamic (safety-preserving)                             |

**Gap SAFEMRS fills**: Code generation is a powerful paradigm (also adopted by Code-as-Symbolic-Planner and AutoMisty) but lacks safety verification. Generated code can contain logical errors, unsafe robot commands, or resource conflicts. SAFEMRS verifies generated code through formal and LLM safety channels before execution.

---

### 2.13 Summary Heatmap

|               | Task Decomp | Dep. Model | Formal Verif. | LLM Safety | CBF Enforce | Conformal | Triple Verif. | MCP/External | RT Monitor | Re-plan | Code Gen | Multi-Agent Dialog | Multi-Formal | VLA Bridge | HAL/Middleware | Robot Resumes |
| ------------- | :---------: | :--------: | :-----------: | :--------: | :---------: | :-------: | :-----------: | :----------: | :--------: | :-----: | :------: | :----------------: | :----------: | :--------: | :------------: | :-----------: |
| SMART-LLM     |     🟢      |     🔴     |      🔴       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| COHERENT      |     🟢      |     🔴     |      🔴       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🟡    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| DART-LLM      |     🟢      |     🟢     |      🔴       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| LaMMA-P       |     🟢      |     🔴     |      🟡       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| LiP-LLM       |     🟢      |     🟢     |      🔴       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| DEXTER-LLM    |     🟢      |     🔴     |      🔴       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🟡     |   🟢    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| SafePlan      |     🔴      |     🔴     |      🔴       |     🟢     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| VerifyLLM     |     🔴      |     🔴     |      🟢       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| SAFER         |     🟢      |     🔴     |      🔴       |     🔴     |     🟢      |    🔴     |      🔴       |      🔴      |     🟡     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| S-ATLAS       |     🟢      |     🔴     |      🔴       |     🔴     |     🔴      |    🟢     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| LLM-CBT       |     🟢      |     🔴     |      🔴       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🟡     |   🟢    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| RoCo          |     🟢      |     🔴     |      🔴       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🟢         |      🔴      |     🔴     |       🔴       |      🔴       |
| Code as Pol.  |     🔴      |     🔴     |      🔴       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🟢    |         🔴         |      🔴      |     🟡     |       🔴       |      🔴       |
| NL2HLTL2PLAN  |     🟢      |     🔴     |      🟢       |     🔴     |     🔴      |    🔴     |      🔴       |      🔴      |     🔴     |   🔴    |    🔴    |         🔴         |      🔴      |     🔴     |       🔴       |      🔴       |
| **SAFEMRS**   |     🟢      |     🟢     |      🟢       |     🟢     |     🟢      |    🟢     |      🟢       |      🟢      |     🟢     |   🟢    |    🟢    |         🟢         |      🟢      |     🟢     |       🟢       |      🟢       |

🟢 = Full support | 🟡 = Partial | 🔴 = Not supported

---

## 3. How to Make the Contribution Substantial (Not Incremental)

> [!IMPORTANT]
> The difference between an incremental and a substantial paper is **depth, not breadth**. A paper that goes deep on one novel idea beats a paper that shallowly combines ten existing ideas. Below are concrete strategies.

---

### Strategy A: Deep-Dive on Corroborative Triple-Channel Verification (Recommended)

**Core idea**: Don't just _combine_ formal verification, LLM CoT, and CBF enforcement — **prove theoretically and demonstrate empirically that no subset of channels is sufficient, and that the triple-channel fusion provides provably better safety properties than any dual-channel combination**.

#### What to do:

1. **Formalize the Corroborative Fusion** as a decision-theoretic framework:
   - Define a **safety lattice** where the formal channel provides a _sound but incomplete_ safety verdict (it catches violations of specified LTL/CTL properties but misses unspecified common-sense hazards)
   - Define the LLM channel as _complete but unsound_ (it can reason about novel hazards but may hallucinate safety)
   - Define the CBF channel as _runtime-enforcing but plan-agnostic_ (it maintains safety sets but cannot evaluate plan logic)
   - Prove that the triple-channel fusion is **strictly safer** than any single or dual-channel combination
   - Incorporate **conformal prediction** (from S-ATLAS) to calibrate confidence scores across channels

2. **Introduce a formal safety metric**, e.g.:
   - **Safety Coverage** = % of hazard types detectable (across specification-level, common-sense, and runtime hazards)
   - **Safety Soundness** = % of "safe" verdicts that are actually safe
   - **Safety Completeness** = % of actually safe plans that are approved
   - **Runtime Violation Rate** = % of plans that violate CBF constraints during execution
   - Show theoretically and empirically that triple-channel dominates dual- and single-channel on the coverage-soundness Pareto frontier

3. **Create a new safety-focused benchmark** — the **Multi-Robot Safety Challenge (MRSC)**:
   - 500+ scenarios across 7 categories: (i) resource conflicts, (ii) spatial collisions, (iii) temporal deadline violations, (iv) common-sense hazards (LLM-detectable only), (v) constraint violations (formal-logic-detectable only), (vi) runtime perturbations (CBF-detectable only), (vii) combined multi-channel hazards
   - Show that **no single-channel system** scores above 50% across all categories, **no dual-channel system** above 75%, while triple-channel scores 90%+
   - Leverage existing benchmarks (RoCoBench, LEMMA, OBiMan-Bench) for task diversity

4. **Ablation study** showing the failure modes of each channel in isolation and in pairs:
   - Cases where formal verification says "safe" but LLM catches a common-sense hazard
   - Cases where LLM says "safe" but formal logic catches a temporal/resource violation
   - Cases where both formal + LLM say "safe" but CBF detects a runtime collision trajectory
   - Cases where CBF enforcement is insufficient without plan-level logical verification

> [!TIP]
> **This strategy makes the contribution non-incremental because**: (a) the theoretical framework for triple-channel fusion is genuinely new (SAFER has CBFs but no formal logic; SafePlan has LLM reasoning but no CBFs; VerifyLLM has formal logic but no LLM safety), (b) the benchmark is a standalone contribution, (c) conformal prediction calibration across channels has never been attempted.

---

### Strategy B: Agentic Planning with MCP as a New Paradigm

**Core idea**: Position the paper around a **new paradigm for grounded multi-robot planning** — the LLM doesn't just decompose tasks from its parametric knowledge; it autonomously retrieves real-world context, calls specialized tools, and refines plans through an agentic loop.

#### What to do:

1. **Formally define "Agentic Multi-Robot Planning"** as a new problem formulation:
   - Standard MRS planning assumes a fixed world model M
   - Agentic MRS planning assumes an **incomplete world model** M₀ that the planner augments through tool calls to produce M₁, M₂, …, Mₖ before planning
   - This is a **different problem class** from all prior work

2. **Design experiments that demonstrate grounding reduces hallucination**:
   - Compare: (a) LLM plans from parametric knowledge only vs. (b) LLM plans with MCP-augmented context
   - Use realistic scenarios: "Deploy robots to inspect the building" where the LLM needs to query a floor plan API, check robot battery levels via a database, and verify weather conditions via a web service
   - Measure: plan correctness, hallucination rate, task feasibility

3. **Show that MCP integration enables qualitatively new mission types**:
   - Missions that are **impossible** without external knowledge (e.g., "Evacuate the building following the fire safety plan" requires querying the building's safety protocol database)
   - Missions requiring real-time external data (e.g., "Navigate around current construction zones" requires querying a city API)

> [!TIP]
> **This strategy makes the contribution non-incremental because**: it defines a new problem class (agentic planning ≠ standard planning) and shows qualitative capabilities impossible for prior systems.

---

### Strategy C: Safety-Aware Re-Planning under Partial Observability (New Problem)

**Core idea**: Define and solve a **new problem** — how to guarantee safety when re-planning in partially observable environments where the world state is uncertain.

#### What to do:

1. **Formulate the problem mathematically**:
   - Given a POMDP (Partially Observable MDP), a set of LTL safety constraints Φ, and a set of LLM-generated sub-tasks T, find a policy π that maximizes mission completion while satisfying Φ with probability ≥ 1 − ε
   - This is a **new problem formulation** that no prior work addresses

2. **Propose a solution that alternates between**:
   - LLM-based belief state estimation (what the robots collectively believe about the world)
   - Formal verification of the plan under the current belief state
   - Re-planning when the belief state update invalidates safety constraints

3. **Benchmark on scenarios with injected partial observability**:
   - Occluded obstacles, uncertain object locations, robot sensor failures
   - Show that SAFEMRS maintains safety guarantees while DEXTER-LLM (closest competitor) does not

> [!TIP]
> **This strategy makes the contribution non-incremental because**: it identifies and solves a genuinely new problem that combines safety, re-planning, and partial observability.

---

### Strategy D: Combine A + B + Systems Innovations (Most Ambitious — Potential RA-L + IROS)

If targeting a journal paper (RA-L with IROS presentation), combine Strategies A and B with the systems contributions identified in the brainstorming analysis:

1. **Theoretical contribution**: Corroborative triple-channel safety framework with conformal prediction calibration (Strategy A)
2. **Systems contribution**: Agentic MCP-augmented planning pipeline with 8-formalism abstract layer (Strategy B) + Hardware Abstraction Layer (HAL) bridging 6 middleware protocols + VLA execution bridge for fine-grained manipulation + Robot Resumes (URDF-derived capability modeling) for embodiment-aware allocation
3. **Benchmark contribution**: Multi-Robot Safety Challenge + adoption of RoCoBench/LEMMA (from Strategy A)
4. **Empirical contribution**: Comprehensive experiments showing safety improvement over SAFER, S-ATLAS, SafePlan, and VerifyLLM individually
5. **Framing**: Position SAFEMRS as a **verifiable neuro-symbolic cognitive proxy** — merging the cognitive proxy framing with the triple-channel safety contribution

---

## 4. Recommended Paper Structure for IROS/ICRA

Based on what gets accepted at these venues, here is a recommended structure:

```
I.    Introduction (1 page)
      - Problem: LLMs are powerful planners but unsafe + ungrounded
      - Gap: No framework combines formal safety + CBF runtime + LLM reasoning + external grounding
      - Contribution: 3 bullet points (theory + system + benchmark)

II.   Related Work (0.75 pages)
      - LLM-based multi-robot planning (SMART-LLM, COHERENT, DART-LLM, LaMMA-P, RoCo)
      - Safety in LLM planning (SafePlan, VerifyLLM, LTLCodeGen, NL2HLTL2PLAN)
      - Runtime safety enforcement (SAFER, S-ATLAS)
      - Code generation paradigms (Code as Policies, ProgPrompt)
      - Dynamic re-planning (DEXTER-LLM, LLM-CBT)
      - Clear gap statement: no work integrates all three safety channels

III.  Problem Formulation (0.75 pages)
      - Formal definition of the safety-aware agentic planning problem
      - Definitions of safety properties (LTL + deontic + CBF safety sets)
      - Triple-channel verification framework with conformal calibration
      - Complexity analysis

IV.   SAFEMRS Architecture (1.5 pages)
      - System overview (one main figure)
      - Agentic Reasoning Layer + MCP
      - Abstract Planning Layer (8 backends)
      - Triple-Channel Safety Verification (theoretical framework)
      - Real-Time Monitoring with safety-preserving re-planning

V.    Experiments (1.5 pages)
      - Benchmark description (MRSC + RoCoBench + LEMMA)
      - Baselines: SafePlan, VerifyLLM, SAFER, S-ATLAS, COHERENT, DEXTER-LLM, LaMMA-P, RoCo
      - Metrics: SR, Safety Coverage, Safety Soundness, CBF violation rate, Re-planning latency
      - Results tables + triple-channel ablation study

VI.   Discussion & Conclusion (0.5 pages)
      - Key findings
      - Limitations (be honest — reviewers respect it)
      - Future work
```

**Total**: 6 pages (IROS/ICRA limit)

---

## 5. What NOT To Do (Common Rejection Reasons)

> [!WARNING]
> These are the most common reasons multi-robot LLM papers get rejected at IROS/ICRA:

| Mistake                             | Why It Kills the Paper                               | How SAFEMRS Avoids It                                                                    |
| ----------------------------------- | ---------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| **"We combine X + Y + Z"**          | Integration ≠ contribution; reviewers want depth     | Focus on ONE deep contribution (triple-channel safety verification) with supporting components |
| **Simulation only, no real robots** | IROS/ICRA reviewers distrust simulation-only results | Include at least a small real-robot experiment (even 2 robots doing 3 tasks)             |
| **Only GPT-4 tested**               | "What if it only works because GPT-4 is good?"       | Test on 2–3 LLMs (GPT-4, Claude, Llama-3) to show architecture-level contribution        |
| **No baselines**                    | Can't judge significance without comparison          | Compare against SafePlan, VerifyLLM, SAFER, S-ATLAS, COHERENT, RoCo on same benchmark    |
| **Vague safety claims**             | "Our system is safer" without quantification         | Define formal safety metrics and measure across triple-channel ablation                   |
| **Too many components, no depth**   | Breadth without depth feels like a tech report       | Cut to 4 key components; give math for triple-channel fusion                              |
| **No failure analysis**             | Reviewers want to see where the system fails         | Include a "Limitations and Failure Cases" section with channel-specific failure modes     |

---

## 6. Concrete Action Plan

### Phase 1: Sharpen the Contribution (1–2 weeks)

- [ ] Choose Strategy A, B, or C as the **primary** contribution
- [ ] Write the formal problem definition (Section III of the paper)
- [ ] Design the corroborative triple-channel safety framework with mathematical rigor
- [ ] Formalize the conformal prediction calibration mechanism
- [ ] Design the benchmark (500+ scenarios with ground-truth safety labels + existing benchmarks)

### Phase 2: Implementation (3–4 weeks)

- [ ] Implement the Agentic Reasoning Layer with MCP tool integration
- [ ] Implement the Triple-Channel Safety Verification:
  - [ ] Formal Logic Channel (LTL/CTL model checking + syntax guarantees from LTLCodeGen)
  - [ ] Probabilistic LLM Channel (CoT safety reasoning + conformal prediction from S-ATLAS)
  - [ ] CBF Runtime Enforcement Channel (inspired by SAFER)
  - [ ] Corroborative fusion with calibrated confidence scores
- [ ] Implement the Abstract Planning Layer (8 backends: PDDL, BT, DAG, HTN, STL, FSM, Code, YAML)
- [ ] Build the Real-Time Monitoring Layer with PEFA closed-loop and CBF safety margin integration
- [ ] Implement the Hardware Abstraction Layer (HAL) with ROS 1, ROS 2, gRPC, MAVLink, XRCE-DDS, VLA adapters
- [ ] Build Robot Resume generator (URDF → textual capability summary) for embodiment-aware task allocation
- [ ] Integrate with a multi-robot simulator (AI2-THOR, Gazebo, or RoCoBench)

### Phase 3: Experiments (2–3 weeks)

- [ ] Run benchmark experiments with 8+ baselines (SafePlan, VerifyLLM, SAFER, S-ATLAS, COHERENT, DEXTER-LLM, LaMMA-P, RoCo)
- [ ] Full ablation study (remove each channel and planning backend independently)
- [ ] Triple-channel vs. dual-channel vs. single-channel comparison
- [ ] Test on 2–3 LLM backends
- [ ] Small real-robot demonstration (if feasible)
- [ ] Evaluate on RoCoBench, LEMMA, and custom MRSC scenarios
- [ ] Prepare results tables and figures

### Phase 4: Writing (1–2 weeks)

- [ ] Draft following the recommended structure in §4
- [ ] Internal review and iteration
- [ ] Final polish and submission

---

## 7. Final Recommendation

> [!IMPORTANT]
> **Go with Strategy A (Triple-Channel Corroborative Safety Verification) as the lead contribution, framed through the "cognitive proxy" lens from brainstorming.**
>
> **Reasons**:
>
> 1. **Safety is the hottest topic** in LLM robotics right now — SafePlan, VerifyLLM, SAFER, and S-ATLAS all appeared in 2024–2025, signaling intense community interest. The bar has risen significantly.
> 2. **No one has combined formal + LLM + CBF safety** — this is genuinely novel. SAFER has CBFs but no formal logic; SafePlan has LLM reasoning but no runtime enforcement; VerifyLLM has formal logic but no CBF or LLM safety reasoning; S-ATLAS has conformal prediction but no formal logic or runtime enforcement.
> 3. **It's provable** — you can provide theoretical guarantees about the triple-channel framework, including conformal prediction calibration bounds, which IROS/ICRA reviewers value.
> 4. **The benchmark is a standalone contribution** — a high-quality multi-robot safety benchmark with 7 hazard categories (leveraging RoCoBench, LEMMA, and custom MRSC) will be valued.
> 5. **MCP integration, HAL, and VLA (Strategy B + systems innovations) support the narrative** — demonstrate them, but don't make them the headline.
> 6. **The expanded landscape (46 papers) strengthens the positioning** — with 14 systems in the comparison heatmap, no single competitor covers even 4 of 16 features.
> 7. **The "cognitive proxy" framing** (from brainstorming) adds conceptual coherence — SAFEMRS is not just a pipeline of tools, it's a **verifiable mediation proxy** between human intent and robot execution.

The paper title could be:

> **"SAFEMRS: A Verifiable Neuro-Symbolic Cognitive Proxy for Safe Multi-Robot Autonomy with Triple-Channel Safety Verification"**

This title signals: (1) a cognitive proxy architecture (systems framing), (2) triple-channel safety verification (primary contribution), (3) multi-robot autonomy (application domain), and (4) neuro-symbolic integration (methodology) — merging the brainstorming vision with the competitive analysis positioning.
