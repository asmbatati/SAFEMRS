# Literature Summary: LLM-Based Multi-Robot Task Planning

> **46 papers analyzed** | Spanning 2020–2025 | Focus: Large Language Models for Multi-Robot Coordination, Task Planning, Safety, and Verification

---

## 1. Overview

This document summarizes 46 research papers investigating the intersection of **Large Language Models (LLMs)** and **multi-robot systems (MRS)**. The literature reveals a rapidly maturing field where LLMs are combined with classical planning formalisms (PDDL, HTN, LTL), optimization methods (LP, MILP), structured representations (DAGs, behavior trees, knowledge graphs), safety mechanisms (CBFs, conformal prediction, formal logic), code generation paradigms, and multi-agent reinforcement learning to achieve robust task decomposition, allocation, coordination, and execution for heterogeneous robot teams.

---

## 2. Paper-by-Paper Summaries

### 2.1 Webster et al. (2020) — Corroborative V&V of Human–Robot Teams

| Field          | Detail                                                                                                                                             |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**      | A Corroborative Approach to Verification and Validation of Human–Robot Teams                                                                       |
| **Venue**      | International Journal of Robotics Research                                                                                                         |
| **Key Idea**   | Combines **formal verification**, **simulation-based testing**, and **user validation** into a corroborative V&V methodology for human–robot teams |
| **Method**     | Uses model checking (PRISM), agent-based simulation (Gazebo), and user studies; evaluates trade-offs between realism and coverability              |
| **Case Study** | Handover task between a human and a robot arm                                                                                                      |
| **Relevance**  | Establishes the need for multi-faceted verification of robot plans — a theme echoed by later LLM-based verification works                          |

---

### 2.2 Pellier et al. (2023) — HDDL 2.1: Temporal HTN Planning

| Field         | Detail                                                                                                                                             |
| ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**     | HDDL 2.1: Towards Defining a Formalism and a Semantics for Temporal HTN Planning                                                                   |
| **Venue**     | ICAPS Workshop                                                                                                                                     |
| **Key Idea**  | Extends PDDL/HDDL to support **temporal and numerical constraints** within Hierarchical Task Network (HTN) planning                                |
| **Method**    | Proposes syntax and semantics for durative actions, temporal ordering (before, after, between), and numeric fluents in hierarchical decompositions |
| **Relevance** | Provides the formal planning language foundation that several LLM-based frameworks (PLANTOR, GMATP-LLM, LaMMA-P) build upon or complement          |

---

### 2.3 Zhao et al. (2024) — MultiBotGPT

| Field        | Detail                                                                                                                                                      |
| ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**    | Applying Large Language Model to a Control System for Multi-Robot Task Assignment                                                                           |
| **Venue**    | IEEE Access                                                                                                                                                 |
| **Key Idea** | Integrates **GPT-3.5** into a layered multi-robot control system (LLLM → LCI → LRC) for translating natural language commands into executable UAV/UGV tasks |
| **Results**  | Outperforms BERT-based baselines in task success rates; reduces human operator cognitive load                                                               |
| **Robots**   | UAVs and UGVs in simulated reconnaissance/delivery scenarios                                                                                                |

---

### 2.4 Kannan et al. (2024) — SMART-LLM

| Field          | Detail                                                                                                                                 |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**      | SMART-LLM: Smart Multi-Agent Robot Task Planning using Large Language Models                                                           |
| **Venue**      | IROS 2024                                                                                                                              |
| **Key Idea**   | Uses **few-shot prompting** with LLMs for three-stage multi-robot planning: task decomposition → coalition formation → task allocation |
| **Benchmark**  | Introduces a benchmark dataset for evaluating multi-robot LLM planners                                                                 |
| **Results**    | Demonstrates promising results in both simulation (AI2-THOR) and real-world scenarios                                                  |
| **Limitation** | Coarse task decomposition granularity; struggles with complex interdependencies                                                        |

---

### 2.5 Saccon et al. (2025) — PLANTOR

| Field                 | Detail                                                                                                                                        |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**             | A Temporal Planning Framework for Multi-Agent Systems via LLM-Aided Knowledge Base Management                                                 |
| **Venue**             | RAL / ICRA 2025                                                                                                                               |
| **Key Idea**          | Integrates LLMs with a **Prolog-based knowledge base** for temporal multi-robot planning                                                      |
| **Method**            | Two-phase KB generation (LLM draft → Prolog validation) + three-step planning (KB query → MILP scheduling → Behavior Tree execution via ROS2) |
| **Temporal Handling** | Explicit durative actions, resource constraints, and parallel execution via MILP optimization                                                 |
| **Results**           | LLMs produce accurate KBs with minimal human feedback in furniture assembly tasks                                                             |

---

### 2.6 Yang et al. (2025) — AutoHMA-LLM

| Field         | Detail                                                                                                                                     |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| **Title**     | AutoHMA-LLM: Efficient Task Coordination and Execution in Heterogeneous Multi-Agent Systems Using Hybrid Large Language Models             |
| **Venue**     | ICRA 2025                                                                                                                                  |
| **Key Idea**  | **Hybrid cloud/device LLM architecture** — a cloud LLM as central planner + device-specific LLMs and Generative Agents for local execution |
| **Scenarios** | Logistics, inspection, and search & rescue with heterogeneous robots                                                                       |
| **Results**   | Improved accuracy, communication efficiency, and reduced token usage compared to fully centralized approaches                              |

---

### 2.7 Liu et al. (2025) — COHERENT

| Field         | Detail                                                                                                                    |
| ------------- | ------------------------------------------------------------------------------------------------------------------------- |
| **Title**     | COHERENT: Collaboration of Heterogeneous Multi-Robot System with Large Language Models                                    |
| **Venue**     | ICRA 2025                                                                                                                 |
| **Key Idea**  | Centralized LLM framework using a **Proposal-Execution-Feedback-Adjustment (PEFA)** mechanism for iterative task planning |
| **Benchmark** | 100 complex long-horizon tasks for heterogeneous multi-robot systems                                                      |
| **Results**   | **97.5% success rate**, surpassing previous methods in both success rate and efficiency                                   |

---

### 2.8 Huang et al. (2025) — LAN2CB

| Field        | Detail                                                                                                 |
| ------------ | ------------------------------------------------------------------------------------------------------ |
| **Title**    | Compositional Coordination for Multi-Robot Teams with Large Language Models                            |
| **Venue**    | ICRA 2025                                                                                              |
| **Key Idea** | Translates **natural language → behavior trees → executable Python code** for multi-robot coordination |
| **Method**   | Mission Analysis module (NL to BT) + Code Generation module (BT to Python)                             |
| **Results**  | Robust coordination demonstrated in both simulation and real-world environments                        |

---

### 2.9 Wang et al. (2025) — DART-LLM

| Field            | Detail                                                                                                         |
| ---------------- | -------------------------------------------------------------------------------------------------------------- |
| **Title**        | DART-LLM: Dependency-Aware Multi-Robot Task Decomposition and Execution using Large Language Models            |
| **Venue**        | ICRA 2025                                                                                                      |
| **Key Idea**     | Uses **Directed Acyclic Graphs (DAGs)** to model task dependencies explicitly                                  |
| **Architecture** | QA LLM (decomposition) → Breakdown Function (robot assignment) → Actuation module → VLM-based object detection |
| **Key Finding**  | Explicit dependency modeling significantly improves smaller LLMs' planning performance                         |
| **Results**      | State-of-the-art performance on multi-robot task benchmarks                                                    |

---

### 2.10 Zhu et al. (2025) — DEXTER-LLM

| Field        | Detail                                                                                                                                |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**    | DEXTER-LLM: Dynamic and Explainable Coordination of Multi-Robot Systems in Unknown Environments via Large Language Models             |
| **Venue**    | Preprint 2025                                                                                                                         |
| **Key Idea** | **Dynamic online planning** in unknown environments with multi-stage LLM reasoning and explainability                                 |
| **Method**   | Mission comprehension → online subtask generation → optimal assignment (branch-and-bound) → dynamic adaptation with human-in-the-loop |
| **Results**  | **100% success rate** in tested scenarios; superior plan quality with explainable outputs                                             |

---

### 2.11 Deng et al. (2025) — GMATP-LLM

| Field        | Detail                                                                                                                                                     |
| ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**    | GMATP-LLM: A General Multi-Agent Task Dynamic Planning Method using Large Language Models                                                                  |
| **Venue**    | Preprint 2025                                                                                                                                              |
| **Key Idea** | Combines **Chain-of-Thought (CoT) prompting** with PDDL planners for multi-agent task planning                                                             |
| **Method**   | LLM-based CoT task decomposition → PDDL goal generation → intelligent planner solver + 3D Spatio-Temporal Motion Corridor for parallel motion optimization |
| **Novelty**  | Integrates spatial-temporal motion planning with high-level task planning                                                                                  |

---

### 2.12 Gupta et al. (2025) — Hierarchical Trees for Multi-Robot Mission Planning

| Field         | Detail                                                                                                  |
| ------------- | ------------------------------------------------------------------------------------------------------- |
| **Title**     | Generalized Mission Planning for Heterogeneous Multi-Robot Teams via LLM-Constructed Hierarchical Trees |
| **Venue**     | ICRA 2025                                                                                               |
| **Key Idea**  | LLM-constructed **hierarchical trees** for mission decomposition with custom APIs and subtree routines  |
| **Method**    | Complex missions → hierarchical sub-task trees → optimized robot schedules via MRTA alternatives        |
| **Strengths** | Flexibility, scalability, and demonstrated generalization across mission types                          |

---

### 2.13 Zhang et al. (2025) — LaMMA-P

| Field            | Detail                                                                                                                                   |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**        | LaMMA-P: Generalizable Multi-Agent Long-Horizon Task Allocation and Planning with LM-Driven PDDL Planner                                 |
| **Venue**        | ICRA 2025                                                                                                                                |
| **Key Idea**     | Integrates LLM reasoning with **PDDL Fast Downward heuristic search** for long-horizon multi-agent planning                              |
| **Architecture** | 6 modules: Precondition Identifier → Task Allocator → Problem Generator → PDDL Validator → Fast Downward/LLM Planner → Sub-Plan Combiner |
| **Benchmark**    | MAT-THOR: 70 tasks across compound, complex, and vague command categories                                                                |
| **Results**      | **105% higher success rate** and **36% higher efficiency** than SMART-LLM baseline                                                       |

---

### 2.14 Obata et al. (2025) — LiP-LLM

| Field             | Detail                                                                                                                 |
| ----------------- | ---------------------------------------------------------------------------------------------------------------------- |
| **Title**         | LiP-LLM: Integrating Linear Programming and Dependency Graph with Large Language Models for Multi-Robot Task Planning  |
| **Venue**         | IEEE RA-L, Feb 2025                                                                                                    |
| **Key Idea**      | Combines **dependency graph generation** (via LLMs) with **linear programming** (LP) for optimal task allocation       |
| **Method**        | 3 steps: skill list generation (SayCan-style likelihood) → DAG dependency graph → LP-based optimal assignment          |
| **Results**       | Maximum success rate difference of **0.82** over baselines; robust across robot constraints and environment complexity |
| **Key Advantage** | LP-based allocation scales better than LLM-based allocation and avoids hallucination in assignment                     |

---

### 2.15 Obi et al. (2025) — SafePlan

| Field          | Detail                                                                                                                                   |
| -------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**      | SafePlan: Leveraging Formal Logic and Chain-of-Thought Reasoning for Enhanced Safety in LLM-based Robotic Task Planning                  |
| **Venue**      | arXiv 2025                                                                                                                               |
| **Key Idea**   | Multi-component **safety verification framework** for LLM-based robotic task planning                                                    |
| **Components** | Prompt Sanity Check COT Reasoner (3 layers: societal → organizational → individual) + Invariant/Precondition/Postcondition COT Reasoners |
| **Benchmark**  | 621 expert-curated task prompts tested on AI2-THOR                                                                                       |
| **Results**    | **90.5% reduction** in harmful task prompt acceptance while maintaining reasonable safe task acceptance                                  |

---

### 2.16 Grigorev et al. (2025) — VerifyLLM

| Field        | Detail                                                                                                                                            |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Title**    | VerifyLLM: LLM-Based Pre-Execution Task Plan Verification for Robots                                                                              |
| **Venue**    | arXiv, Jul 2025                                                                                                                                   |
| **Key Idea** | **Pre-execution verification** of robot task plans by combining LTL formulas with LLM-based contextual reasoning                                  |
| **Method**   | Translation Module (NL → LTL) + Verification Module (sliding window analysis detecting position errors, missing prerequisites, redundant actions) |
| **Datasets** | ALFRED-LTL, VirtualHome-LTL                                                                                                                       |
| **Results**  | ~40% reduction in ordering errors with Claude-based implementation; LLM verification contributes far more than LTL alone                          |

---

### 2.17 Khan et al. (2025) — SAFER

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Safety Aware Task Planning via Large Language Models in Robotics |
| **Venue**    | IROS 2025 |
| **Key Idea** | Multi-LLM **safety pipeline** combining task planning with **Control Barrier Functions (CBFs)** for runtime safety enforcement |
| **Method**   | Safety-Aware LLM generates constraints → CBF layer enforces safety during execution → feedback loop for re-planning |
| **Results**  | Demonstrates safe task execution in scenarios where unconstrained LLM plans would violate safety boundaries |
| **Relevance**| Directly related to SAFEMRS — integrates safety at both planning and execution levels |

---

### 2.18 Tian et al. (2025) — LLM-CBT

| Field        | Detail |
| ------------ | ------ |
| **Title**    | LLM-CBT: LLM-Driven Closed-Loop Behavior Tree Planning for Heterogeneous UAV-UGV Swarm Collaboration |
| **Venue**    | IROS 2025 |
| **Key Idea** | LLM generates **closed-loop behavior trees** for heterogeneous UAV-UGV swarms with dynamic re-planning |
| **Method**   | LLM-based BT generation → closed-loop execution monitoring → dynamic BT modification based on environmental feedback |
| **Robots**   | UAV-UGV swarms in collaborative scenarios |
| **Relevance**| Combines BTs with LLMs for heterogeneous swarm coordination — key SAFEMRS theme |

---

### 2.19 Rabiei et al. (2025) — LTLCodeGen

| Field        | Detail |
| ------------ | ------ |
| **Title**    | LTLCodeGen: Code Generation of Syntactically Correct Temporal Logic for Robot Task Planning |
| **Venue**    | IROS 2025 |
| **Key Idea** | Generates **syntactically correct LTL specifications** from natural language via code generation approach |
| **Method**   | NL → LLM code generation → syntax-guaranteed LTL formulas → classical planner execution |
| **Results**  | Eliminates syntax errors in LTL generation that plague direct LLM translation approaches |
| **Relevance**| Formal verification pipeline for temporal specifications — complements VerifyLLM |

---

### 2.20 Xu et al. (2025) — NL2HLTL2PLAN

| Field        | Detail |
| ------------ | ------ |
| **Title**    | NL2HLTL2PLAN: Scaling Up Natural Language Understanding for Multi-Robots Through Hierarchical Temporal Logic |
| **Venue**    | RA-L 2025 |
| **Key Idea** | **Hierarchical LTL (HLTL)** decomposition for scaling NL-to-formal-specification for multi-robot teams |
| **Method**   | NL → hierarchical LTL synthesis → multi-robot plan generation with compositional task specifications |
| **Results**  | Scales temporal logic planning to larger multi-robot teams than flat LTL approaches |
| **Relevance**| Addresses scalability of formal verification in multi-robot LLM planning |

---

### 2.21 Wang et al. (2025) — S-ATLAS

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Probabilistically Correct Language-Based Multi-Robot Planning Using Conformal Prediction |
| **Venue**    | RA-L 2025 |
| **Key Idea** | Uses **conformal prediction** to provide probabilistic correctness guarantees for LLM-generated multi-robot plans |
| **Method**   | LLM planning → conformal prediction calibration → probabilistic safety bounds on plan success |
| **Results**  | Provides formal probabilistic guarantees without sacrificing planning flexibility |
| **Relevance**| Novel statistical safety mechanism complementing formal verification approaches |

---

### 2.22 Yuan et al. (2025) — Knowledge Graph BT Generation

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Trustworthy Robot Behavior Tree Generation Based on Multi-Source Heterogeneous Knowledge Graph |
| **Venue**    | ICRA 2025 |
| **Key Idea** | Generates **trustworthy behavior trees** from a multi-source heterogeneous **knowledge graph** |
| **Method**   | Knowledge graph construction from heterogeneous sources → graph-guided BT generation → trustworthiness validation |
| **Results**  | Improved BT quality and consistency compared to direct LLM generation |
| **Relevance**| Knowledge-grounded BT generation — addresses hallucination concerns in LLM-based BT synthesis |

---

### 2.23 Yu et al. (2025) — CLGA

| Field        | Detail |
| ------------ | ------ |
| **Title**    | CLGA: A Collaborative LLM Framework for Dynamic Goal Assignment in Multi-Robot Systems |
| **Venue**    | IROS 2025 |
| **Key Idea** | **Dual-process** (fast/slow thinking) LLM framework for dynamic multi-robot goal assignment |
| **Method**   | Slow-thinking LLM (deliberative analysis) + fast-thinking solver (reactive assignment) → collaborative decision-making |
| **Results**  | Balances planning quality with real-time responsiveness for dynamic scenarios |
| **Relevance**| Novel cognitive architecture for multi-robot coordination with LLMs |

---

### 2.24 Wan et al. (2025) — EmbodiedAgent

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Hierarchical Multi-Robot Task Planning with Fine-Tuned LLM |
| **Venue**    | IROS 2025 |
| **Key Idea** | **Fine-tuned LLM** for hierarchical multi-robot embodied task planning |
| **Method**   | Domain-specific LLM fine-tuning → hierarchical task decomposition → multi-robot execution with embodied feedback |
| **Results**  | Fine-tuned models outperform generic LLMs in domain-specific multi-robot scenarios |
| **Relevance**| Demonstrates value of task-specific LLM adaptation for multi-robot planning |

---

### 2.25 Wang et al. (2025) — LLM+Scene Graph+LTL

| Field        | Detail |
| ------------ | ------ |
| **Title**    | LLM-Based Cross-Regional Multi-Robot Task Planning with Scene Graphs and LTL |
| **Venue**    | IROS 2025 |
| **Key Idea** | Integrates **scene graphs** with **LTL specifications** for cross-regional multi-robot planning |
| **Method**   | Scene graph perception → LLM spatial reasoning → LTL constraint generation → multi-region plan synthesis |
| **Results**  | Enables multi-robot coordination across spatially separated environments |
| **Relevance**| Combines perception (scene graphs) with formal methods (LTL) for spatial multi-robot planning |

---

### 2.26 Chen et al. (2025) — MAS for Robotic Autonomy

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Multi-Agent Systems for Robotic Autonomy with Reinforcement Learning |
| **Venue**    | IROS 2025 |
| **Key Idea** | Combines **multi-agent systems** with **reinforcement learning** for enhanced robotic autonomy |
| **Method**   | MAS architecture → RL-based policy learning → cooperative task execution |
| **Results**  | Demonstrates emergent cooperative behaviors through RL-based multi-agent training |
| **Relevance**| Alternative paradigm to pure LLM planning — RL-based multi-agent coordination |

---

### 2.27 Chen et al. (2024) — Code-as-Symbolic-Planner

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Code as Symbolic Planner: LLMs Generate Symbolic Code as TAMP Planners with Multi-Agent Self-Guidance |
| **Venue**    | Preprint 2024 |
| **Key Idea** | LLMs generate **symbolic code** that acts as Task and Motion Planning (TAMP) planners, with **multi-agent self-guidance** |
| **Method**   | Multi-agent LLM system: planner agent generates symbolic code → critic agent validates → executor agent runs planning |
| **Results**  | Symbolic code planners outperform direct LLM planning in complex TAMP scenarios |
| **Relevance**| Code generation as planning formalism — bridges LLM reasoning and classical TAMP |

---

### 2.28 Yano et al. (2025) — ICCO

| Field        | Detail |
| ------------ | ------ |
| **Title**    | ICCO: MARL Coordinator for Language-Guided Multi-Robot Systems with CTDE |
| **Venue**    | Preprint 2025 |
| **Key Idea** | **Multi-Agent RL (MARL)** coordinator using **Centralized Training Decentralized Execution (CTDE)** for language-guided multi-robot systems |
| **Method**   | Language instruction parsing → MARL policy training (centralized) → decentralized execution with local observations |
| **Results**  | Improved coordination efficiency compared to purely LLM-based or purely RL-based approaches |
| **Relevance**| Hybrid LLM+MARL paradigm for multi-robot systems — combines language understanding with learned coordination |

---

### 2.29 Song et al. (2025) — FCRF

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Flexible Constructivism Reflection Framework for Long-Horizon LLM Planning |
| **Venue**    | Preprint 2025 |
| **Key Idea** | **Reflection-based** LLM framework for long-horizon planning using flexible constructivism principles |
| **Method**   | Plan generation → structured self-reflection → plan refinement cycle, inspired by constructivist learning theory |
| **Results**  | Improved long-horizon planning success rates through iterative reflection loops |
| **Relevance**| Self-improvement mechanism for LLM planners — addresses hallucination and error accumulation |

---

### 2.30 Sun et al. (2025) — Survey: LLM-Based MADM

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Survey on LLM-Based Multi-Agent Decision Making: Challenges and Directions |
| **Venue**    | Preprint 2025 |
| **Key Idea** | Comprehensive **survey** of LLM-based multi-agent decision making, categorizing challenges and future directions |
| **Coverage** | Communication protocols, coordination mechanisms, scalability, safety, and real-world deployment barriers |
| **Relevance**| Provides broad landscape context and identifies open challenges relevant to SAFEMRS positioning |

---

### 2.31 Reasoner et al. (2025) — Theory of Mind for MRS

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Theory of Mind and LLM Attention Selection for Implicit Multi-Robot System Coordination |
| **Venue**    | Preprint 2025 |
| **Key Idea** | Uses **Theory of Mind (ToM)** modeling with LLM-based **attention selection** for implicit multi-robot coordination |
| **Method**   | ToM reasoning about other agents' beliefs/goals → LLM attention mechanism for coordination signal selection |
| **Results**  | Enables coordination without explicit communication channels between robots |
| **Relevance**| Novel implicit coordination paradigm — reduces communication overhead in multi-robot teams |

---

### 2.32 Zuzuárregui & Carpin (2025) — Precision Agriculture

| Field        | Detail |
| ------------ | ------ |
| **Title**    | LLM Mission Planning for Precision Agriculture with IEEE Standard |
| **Venue**    | Preprint 2025 |
| **Key Idea** | LLM-based mission planning for **precision agriculture** adhering to **IEEE standard** mission specifications |
| **Method**   | NL agricultural objectives → LLM mission synthesis → IEEE-compliant plan generation → multi-robot farm execution |
| **Results**  | Demonstrated plan generation for agricultural scenarios with standard-compliant outputs |
| **Relevance**| Domain-specific application of LLM planning with formal standard compliance |

---

### 2.33 Mandi et al. (2024) — RoCo

| Field        | Detail |
| ------------ | ------ |
| **Title**    | RoCo: Dialectic Multi-Robot Collaboration with LLM Negotiations |
| **Venue**    | ICRA 2024 |
| **Key Idea** | **Dialectic collaboration** — robots negotiate via LLM-generated dialog for task coordination and motion planning |
| **Method**   | Multi-LLM dialog → consensus-based task allocation → coordinated motion planning → RoCoBench benchmark |
| **Benchmark**| **RoCoBench**: standardized benchmark for multi-robot collaborative manipulation |
| **Relevance**| Inter-robot communication via LLMs — enables emergent collaborative strategies |

---

### 2.34 Chen et al. (2024) — AutoTAMP

| Field        | Detail |
| ------------ | ------ |
| **Title**    | AutoTAMP: LLM as Translator from Natural Language to Signal Temporal Logic for TAMP |
| **Venue**    | Preprint 2024 |
| **Key Idea** | LLM translates NL to **Signal Temporal Logic (STL)** with **autoregressive syntactic+semantic checking** |
| **Method**   | NL → LLM STL generation → autoregressive syntax validation → semantic compatibility checking → TAMP execution |
| **Results**  | Produces formally valid STL specifications from natural language with high accuracy |
| **Relevance**| Extends formal specification generation beyond LTL to continuous-time STL — richer temporal expressiveness |

---

### 2.35 Gong et al. (2023) — LEMMA

| Field        | Detail |
| ------------ | ------ |
| **Title**    | LEMMA: Benchmark for Language-Conditioned Multi-Robot Manipulation |
| **Venue**    | CoRL 2023 |
| **Key Idea** | **Benchmark** for evaluating language-conditioned multi-robot manipulation with heterogeneous robots |
| **Benchmark**| Diverse tasks requiring coordination between different robot types (arms, grippers, mobile) |
| **Metrics**  | Success rate, coordination quality, language grounding accuracy, efficiency |
| **Relevance**| Standardized evaluation framework — addresses the benchmark gap in multi-robot LLM planning |

---

### 2.36 Wang et al. (2025) — AutoMisty

| Field        | Detail |
| ------------ | ------ |
| **Title**    | AutoMisty: Multi-Agent LLM Framework for Code Generation on Social Robot |
| **Venue**    | Preprint 2025 |
| **Key Idea** | **Multi-agent LLM framework** for generating robot code with **two-layer optimization** |
| **Method**   | Planner LLM → Coder LLM → Reviewer LLM, with inner (code quality) and outer (task performance) optimization loops |
| **Results**  | Improved code generation quality and task success on social robot platforms |
| **Relevance**| Multi-agent LLM self-improvement pattern for code generation in robotics |

---

### 2.37 Kwon et al. (2025) — Hierarchical Scene Graph Planning

| Field        | Detail |
| ------------ | ------ |
| **Title**    | LLM-Based Task Planning for Abstract Commands Using Hierarchical Scene Graphs |
| **Venue**    | Preprint 2025 |
| **Key Idea** | **Hierarchical scene graphs** enable LLM task planning for **abstract, underspecified commands** |
| **Method**   | Scene graph hierarchy construction → LLM grounding of abstract commands → concrete plan generation |
| **Results**  | Handles vague commands ("clean up the room") by leveraging structured scene understanding |
| **Relevance**| Addresses the abstraction gap between human commands and robot-executable plans |

---

### 2.38 Hoffmeister et al. (2025) — Zero-Knowledge BT Generation

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Zero-Knowledge Task Planning Using LLM-Generated Behavior Trees with Refinement Loop |
| **Venue**    | Preprint 2025 |
| **Key Idea** | **Zero-knowledge** BT generation — LLM generates behavior trees without prior domain knowledge, using iterative **refinement loops** |
| **Method**   | LLM BT generation → execution attempt → failure analysis → BT refinement → re-execution |
| **Results**  | Achieves task completion through iterative refinement without requiring pre-defined action libraries |
| **Relevance**| Minimizes domain knowledge requirements — enables rapid deployment to new domains |

---

### 2.39 Liang et al. (2023) — Code as Policies

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Code as Policies: Language Model Programs for Embodied Control |
| **Venue**    | ICRA 2023 |
| **Key Idea** | LLMs generate **Python policy code** from natural language, enabling spatial-geometric reasoning and generalization |
| **Method**   | Hierarchical code generation: high-level policy code → API calls to perception/control primitives → robot execution |
| **Results**  | Generalizes to novel objects and spatial configurations through code composition |
| **Relevance**| Foundational work — establishes code generation as a robot control paradigm for LLMs |

---

### 2.40 Singh et al. (2023) — ProgPrompt

| Field        | Detail |
| ------------ | ------ |
| **Title**    | ProgPrompt: Generating Situated Robot Task Plans Using Large Language Models |
| **Venue**    | ICRA 2023 |
| **Key Idea** | Uses **programming language structures** in prompts for situated task planning with **state feedback assertions** |
| **Method**   | Programmatic prompt formatting → LLM plan generation → assertion-based state validation → re-planning on failure |
| **Results**  | State-aware planning reduces execution failures in virtual household environments |
| **Relevance**| Programmatic prompting paradigm — structured code-like prompts improve LLM plan quality |

---

### 2.41 Ding et al. (2023) — LLM-GROP

| Field        | Detail |
| ------------ | ------ |
| **Title**    | LLM-GROP: Task and Motion Planning with Large Language Models for Object Rearrangement |
| **Venue**    | IROS 2023 |
| **Key Idea** | Extracts **commonsense knowledge** about object configurations from LLMs and instantiates with a **TAMP system** |
| **Method**   | LLM commonsense extraction → spatial relationship reasoning → TAMP solver for feasible placements |
| **Results**  | Generalizes to novel object rearrangement scenarios using LLM-derived commonsense |
| **Relevance**| Commonsense grounding for TAMP — LLM as knowledge source rather than planner |

---

### 2.42 Chu et al. (2024) — LABOR

| Field        | Detail |
| ------------ | ------ |
| **Title**    | LABOR: Large Language Models for Orchestrating Bimanual Robots |
| **Venue**    | Humanoids 2024 |
| **Key Idea** | LLM agent analyzes task configurations and devises **coordination control policies** for **bimanual** tasks |
| **Method**   | Task analysis → LLM coordination policy generation → spatio-temporal control pattern selection → bimanual execution |
| **Results**  | Effective coordination in 6 bimanual manipulation tasks requiring tight synchronization |
| **Relevance**| Extends LLM planning to fine-grained bimanual coordination — tight coupling between arms |

---

### 2.43 Mu et al. (2025) — Look Before You Leap (FSM-based)

| Field        | Detail |
| ------------ | ------ |
| **Title**    | Look Before You Leap: Using Serialized State Machine for Language Conditioned Robotic Manipulation |
| **Venue**    | IROS 2025 |
| **Key Idea** | Uses **serialized Finite State Machines (FSMs)** to generate demonstrations for language-conditioned manipulation |
| **Method**   | Language instruction → FSM-guided state transitions → demonstration generation → imitation learning |
| **Results**  | Improved success rates in long-horizon manipulation tasks compared to direct policy learning |
| **Relevance**| FSM as structured intermediate representation — bridges language and low-level manipulation |

---

### 2.44 Tang et al. (2025) — OBiMan-Bench & OBiMan-Planner

| Field        | Detail |
| ------------ | ------ |
| **Title**    | OBiMan-Bench & OBiMan-Planner: Open-World Task Planning for Humanoid Bimanual Dexterous Manipulation via VLMs |
| **Venue**    | Preprint 2025 |
| **Key Idea** | **Vision-Language Model (VLM)** based zero-shot planning for **bimanual dexterous manipulation** with benchmark |
| **Benchmark**| OBiMan-Bench: evaluates open-world planning across diverse bimanual manipulation scenarios |
| **Method**   | VLM visual grounding → task decomposition → bimanual coordination plan → execution |
| **Relevance**| Extends LLM planning to VLM-based visual grounding for complex dexterous manipulation |

---

## 3. Thematic Analysis

### 3.1 LLM Integration Patterns

The literature reveals **seven dominant patterns** for integrating LLMs into robot planning:

| Pattern                     | Description                                                            | Papers                                                                     |
| --------------------------- | ---------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| **LLM as Decomposer**       | LLM decomposes NL instructions into sub-tasks                          | SMART-LLM, DART-LLM, COHERENT, LaMMA-P, LiP-LLM, GMATP-LLM, CLGA, EmbodiedAgent |
| **LLM + Classical Planner** | LLM generates formal specs solved by traditional planners              | LaMMA-P (PDDL), GMATP-LLM (PDDL), PLANTOR (MILP), LiP-LLM (LP), AutoTAMP (STL) |
| **LLM as Code Generator**   | LLM produces executable code, BTs, or policies                        | LAN2CB, Code as Policies, ProgPrompt, Code-as-Symbolic-Planner, AutoMisty |
| **LLM as Verifier**         | LLM validates plans for correctness and safety                         | VerifyLLM, SafePlan, SAFER (CBFs), S-ATLAS (conformal prediction)          |
| **LLM + Formal Logic**      | LLM translates NL to temporal logic specifications                     | LTLCodeGen (LTL), NL2HLTL2PLAN (HLTL), AutoTAMP (STL), Wang+LTL          |
| **LLM as Negotiator**       | Multi-LLM dialog for collaborative planning                           | RoCo, CLGA (dual-process), DEXTER-LLM (human-in-the-loop)                |
| **LLM + Knowledge Structures** | LLM combined with knowledge graphs, scene graphs, or ontologies    | Yuan KG-BT, Kwon (scene graphs), Wang+SceneGraph+LTL, PLANTOR (Prolog KB) |

### 3.2 Dependency and Temporal Modeling

A critical recurring challenge is correctly modeling **task dependencies and temporal constraints**:

- **DAG-based**: DART-LLM, LiP-LLM explicitly use directed acyclic graphs
- **PDDL temporal extensions**: HDDL 2.1, PLANTOR, GMATP-LLM
- **LTL-based**: VerifyLLM, SafePlan, LTLCodeGen, NL2HLTL2PLAN, Wang+SceneGraph+LTL
- **STL-based**: AutoTAMP extends to continuous-time Signal Temporal Logic
- **Hierarchical decomposition**: Gupta et al., LaMMA-P, AutoHMA-LLM, EmbodiedAgent, Kwon
- **FSM-based**: Mu et al. (Look Before You Leap) uses serialized FSMs

### 3.3 Heterogeneous Robot Teams

Most frameworks explicitly target **heterogeneous multi-robot systems** where robots possess different skills:

- Skill-aware allocation: SMART-LLM, LaMMA-P, LiP-LLM, COHERENT, EmbodiedAgent
- Cloud/device hybrid architectures: AutoHMA-LLM
- Capability-conditioned planning: DART-LLM, Gupta et al., DEXTER-LLM
- UAV-UGV swarm coordination: LLM-CBT, MultiBotGPT
- Bimanual coordination: LABOR, OBiMan-Planner
- Heterogeneous benchmarks: LEMMA (multi-type robots)

### 3.4 Safety and Verification

A growing sub-field addresses plan **safety and correctness** — now the largest thematic cluster:

| Approach                       | Mechanism                                       | Paper(s)                |
| ------------------------------ | ----------------------------------------------- | ----------------------- |
| Pre-execution verification     | LTL + LLM sliding window                        | VerifyLLM               |
| Prompt safety screening        | Formal logic + CoT multi-layer reasoning         | SafePlan                |
| Corroborative V&V              | Model checking + simulation + user studies       | Webster et al.          |
| Human-in-the-loop              | Dynamic adaptation with explainability           | DEXTER-LLM              |
| Runtime safety enforcement     | Control Barrier Functions (CBFs)                 | SAFER                   |
| Probabilistic guarantees       | Conformal prediction calibration                 | S-ATLAS                 |
| Syntactic correctness          | Code-generation-based LTL synthesis              | LTLCodeGen              |
| Knowledge-grounded trust       | Multi-source KG validation                       | Yuan et al.             |

### 3.5 Code Generation and Programmatic Planning

An emerging paradigm uses **code as the planning representation**:

- **Policy code generation**: Code as Policies — LLMs generate Python control code
- **Programmatic prompting**: ProgPrompt — program-structured prompts with state assertions
- **Symbolic code planners**: Code-as-Symbolic-Planner — LLM-generated code as TAMP solvers
- **BT code generation**: LAN2CB, LLM-CBT, Hoffmeister (zero-knowledge BT refinement)
- **Multi-agent code review**: AutoMisty — planner/coder/reviewer LLM pipeline

### 3.6 Multi-Agent LLM Architectures

Several works explore **multi-LLM collaboration** patterns for improving planning quality:

- **Dialog-based**: RoCo (inter-robot LLM negotiation)
- **Dual-process**: CLGA (fast solver + slow LLM reasoning)
- **Self-guidance**: Code-as-Symbolic-Planner (planner + critic + executor agents)
- **Reflection-based**: FCRF (constructivist self-reflection loops)
- **Theory of Mind**: Reasoner et al. (implicit coordination via ToM modeling)

### 3.7 Benchmarks and Evaluation

| Benchmark         | Source          | Tasks                               | Papers Using It                                        |
| ----------------- | --------------- | ----------------------------------- | ------------------------------------------------------ |
| AI2-THOR          | Allen AI        | Household simulation                | SMART-LLM, LaMMA-P, COHERENT, SafePlan                 |
| MAT-THOR          | Zhang et al.    | 70 long-horizon multi-agent tasks   | LaMMA-P                                                |
| VirtualHome       | Puig et al.     | Daily household activities          | VerifyLLM, ProgPrompt                                  |
| ALFRED            | Shridhar et al. | Vision-language navigation          | VerifyLLM                                              |
| RoCoBench         | Mandi et al.    | Multi-robot collaborative manip.    | RoCo                                                   |
| LEMMA             | Gong et al.     | Language-conditioned multi-robot    | LEMMA                                                  |
| OBiMan-Bench      | Tang et al.     | Bimanual dexterous manipulation     | OBiMan-Planner                                         |
| Custom benchmarks | Various         | Domain-specific                     | DART-LLM, COHERENT (100), SafePlan (621), SAFER        |

---

## 4. Comparative Summary Table

| Paper              | Year | LLM Role               | Planning Formalism     | Optimization       | Robots           | Key Metric                    |
| ------------------ | ---- | ---------------------- | ---------------------- | ------------------ | ---------------- | ----------------------------- |
| Webster et al.     | 2020 | —                      | Model checking         | —                  | HRI team         | V&V coverage                  |
| Pellier et al.     | 2023 | —                      | HDDL 2.1 / HTN         | —                  | Multi-agent      | Formalism completeness        |
| Code as Policies   | 2023 | Code generator         | Python policies        | —                  | Single arm       | Generalization                |
| ProgPrompt         | 2023 | Programmatic prompts   | Assertions + code      | —                  | Single robot     | State-aware SR                |
| LLM-GROP           | 2023 | Knowledge source       | TAMP                   | Spatial reasoning  | Single arm       | Commonsense transfer          |
| LEMMA              | 2023 | —                      | Benchmark              | —                  | Heterogeneous    | Benchmark metrics             |
| MultiBotGPT        | 2024 | Controller             | Layered architecture   | —                  | UAV + UGV        | Task success rate             |
| SMART-LLM          | 2024 | Decomposer             | Few-shot prompting     | —                  | Heterogeneous    | SR, GCR, Exe                  |
| RoCo               | 2024 | Dialog negotiator      | Multi-LLM consensus    | Motion planning    | Multi-arm        | Collaboration SR              |
| Code-as-Symb.      | 2024 | Symbolic code gen.     | TAMP via code          | —                  | Multi-agent      | TAMP quality                  |
| AutoTAMP           | 2024 | NL→STL translator      | Signal Temporal Logic  | —                  | Single/multi     | STL validity                  |
| LABOR              | 2024 | Coordination policy    | Bimanual control       | —                  | Dual arm         | Bimanual SR                   |
| PLANTOR            | 2025 | KB manager             | Prolog + MILP          | MILP               | Multi-arm        | KB accuracy                   |
| AutoHMA-LLM        | 2025 | Coordinator            | Hybrid cloud/device    | —                  | Heterogeneous    | Token efficiency              |
| COHERENT           | 2025 | Planner (PEFA)         | Centralized LLM        | —                  | Heterogeneous    | **97.5% SR**                  |
| LAN2CB             | 2025 | Code generator         | Behavior trees         | —                  | Multi-robot      | Execution robustness          |
| DART-LLM           | 2025 | Decomposer             | DAG dependencies       | —                  | Multi-robot      | SOTA SR                       |
| DEXTER-LLM         | 2025 | Online planner         | Branch-and-bound       | B&B                | Multi-robot      | **100% SR**                   |
| GMATP-LLM          | 2025 | Decomposer (CoT)       | PDDL                   | Motion corridor    | Multi-agent      | Plan quality                  |
| Gupta et al.       | 2025 | Tree constructor       | Hierarchical trees     | MRTA               | Heterogeneous    | Scalability                   |
| LaMMA-P            | 2025 | Decomp. + validator    | PDDL + Fast Downward   | Heuristic search   | Heterogeneous    | **105% SR improvement**       |
| LiP-LLM            | 2025 | Graph generator        | DAG + LP               | Linear programming | Arm + mobile     | **0.82 SR diff.**             |
| SafePlan           | 2025 | Safety verifier        | Formal logic + CoT     | —                  | Multi-robot      | **90.5% harm reduction**      |
| VerifyLLM          | 2025 | Plan verifier          | LTL + LLM              | —                  | Single/multi     | **40% order error reduction** |
| SAFER              | 2025 | Safety pipeline        | LLM + CBFs             | CBF optimization   | Multi-robot      | Safety compliance             |
| LLM-CBT            | 2025 | BT generator           | Closed-loop BTs        | —                  | UAV-UGV swarm    | Swarm coordination            |
| LTLCodeGen         | 2025 | Code→LTL generator     | LTL via code gen.      | —                  | Single/multi     | Syntax correctness            |
| NL2HLTL2PLAN       | 2025 | NL→HLTL translator     | Hierarchical LTL       | —                  | Multi-robot      | Scalability                   |
| S-ATLAS            | 2025 | Probabilistic planner  | Conformal prediction   | —                  | Multi-robot      | Probabilistic guarantees      |
| Yuan et al.        | 2025 | KG→BT generator        | Knowledge graph + BT   | —                  | Single robot     | BT trustworthiness            |
| CLGA               | 2025 | Dual-process assigner  | LLM + solver           | —                  | Multi-robot      | Responsiveness                |
| EmbodiedAgent      | 2025 | Fine-tuned planner     | Hierarchical decomp.   | —                  | Multi-robot      | Domain-specific SR            |
| Wang+SG+LTL        | 2025 | Spatial reasoner       | Scene graph + LTL      | —                  | Multi-robot      | Cross-regional planning       |
| Chen MAS+RL        | 2025 | —                      | MARL                   | RL optimization    | Multi-agent      | Emergent cooperation          |
| ICCO               | 2025 | Language parser         | MARL (CTDE)            | RL coordination    | Multi-robot      | Coordination efficiency       |
| FCRF               | 2025 | Reflective planner     | Self-reflection loops  | —                  | Single/multi     | Long-horizon SR               |
| Sun Survey         | 2025 | — (survey)             | —                      | —                  | Multi-agent      | Landscape overview            |
| Reasoner ToM       | 2025 | ToM modeler            | Implicit coordination  | —                  | Multi-robot      | Communication reduction       |
| Zuzuárregui        | 2025 | Mission planner        | IEEE standard          | —                  | Agricultural     | Standard compliance           |
| AutoMisty          | 2025 | Multi-agent code gen.  | Two-layer optimization | Code optimization  | Social robot     | Code quality                  |
| Kwon               | 2025 | Scene-aware planner    | Hierarchical SG        | —                  | Single robot     | Abstract command handling      |
| Hoffmeister        | 2025 | Zero-knowledge BT gen. | BT + refinement loop   | —                  | Single robot     | Zero-knowledge SR             |
| Mu et al.          | 2025 | FSM generator          | Serialized FSM         | Imitation learning | Single arm       | Long-horizon SR               |
| OBiMan             | 2025 | VLM planner            | VLM grounding          | —                  | Bimanual humanoid| Open-world SR                 |

---

## 5. Key Research Gaps and Future Directions

1. **Real-world deployment**: Most frameworks are validated only in simulation (AI2-THOR, VirtualHome). Real-world transfer with sensor noise and dynamic environments remains underexplored. Only SAFER, RoCo, and DEXTER-LLM show partial real-world validation.

2. **Scalability**: Few papers demonstrate performance beyond 3–5 robots. LiP-LLM, AutoHMA-LLM, and NL2HLTL2PLAN discuss scalability but lack large-scale (>10 robot) validation.

3. **Safety guarantees**: The safety landscape has expanded significantly (SAFER/CBFs, S-ATLAS/conformal prediction, SafePlan/formal logic, LTLCodeGen/syntax guarantees), but **end-to-end formal safety guarantees** for LLM-generated plans remain an open problem — a core opportunity for SAFEMRS.

4. **Temporal reasoning**: HDDL 2.1, PLANTOR, AutoTAMP (STL), and NL2HLTL2PLAN address temporal planning, but most LLM-based frameworks treat time simplistically or ignore it.

5. **Dynamic re-planning**: DEXTER-LLM, LLM-CBT, and CLGA handle dynamic environments, but robust online re-planning with safety preservation remains challenging.

6. **Unified benchmarks**: RoCoBench, LEMMA, and OBiMan-Bench are important steps, but the field still lacks a standardized multi-robot benchmark spanning diverse scenarios, robot types, and complexity levels.

7. **Explainability and trust**: DEXTER-LLM and FCRF emphasize explainability, but most frameworks provide limited insight into reasoning. Theory of Mind approaches (Reasoner et al.) open new directions.

8. **Multi-modal grounding**: Few works integrate visual perception with LLM planning. OBiMan-Planner (VLMs), Kwon (scene graphs), and Wang+SceneGraph+LTL are early examples of perception-integrated planning that need further development.

9. **Bimanual and dexterous manipulation**: LABOR and OBiMan represent an emerging frontier where LLM planning must handle fine-grained coordination — scaling these approaches to multi-robot teams is unexplored.

10. **Hybrid LLM+RL paradigms**: ICCO and Chen MAS+RL demonstrate that combining LLMs with learned policies can improve coordination, but principled integration frameworks are lacking.

---

## 6. References

1. Webster, M. et al. (2020). "A corroborative approach to verification and validation of human–robot teams." _IJRR_.
2. Pellier, D. et al. (2023). "HDDL 2.1: Towards Defining a Formalism and a Semantics for Temporal HTN Planning." _ICAPS Workshop_.
3. Liang, J. et al. (2023). "Code as Policies: Language Model Programs for Embodied Control." _ICRA 2023_.
4. Singh, I. et al. (2023). "ProgPrompt: Generating Situated Robot Task Plans using Large Language Models." _ICRA 2023_.
5. Ding, Y. et al. (2023). "LLM-GROP: Task and Motion Planning with Large Language Models for Object Rearrangement." _IROS 2023_.
6. Gong, D. et al. (2023). "LEMMA: Benchmark for Language-Conditioned Multi-Robot Manipulation." _CoRL 2023_.
7. Zhao, J. et al. (2024). "Applying Large Language Model to a Control System for Multi-Robot Task Assignment." _IEEE Access_.
8. Kannan, S. et al. (2024). "SMART-LLM: Smart Multi-Agent Robot Task Planning using Large Language Models." _IROS 2024_.
9. Mandi, Z. et al. (2024). "RoCo: Dialectic Multi-Robot Collaboration with Large Language Model Negotiations." _ICRA 2024_.
10. Chen, Y. et al. (2024). "Code as Symbolic Planner: LLMs Generate Symbolic Code as TAMP Planners." _Preprint_.
11. Chen, Y. et al. (2024). "AutoTAMP: LLM as Translator from Natural Language to Signal Temporal Logic for TAMP." _Preprint_.
12. Chu, Y. et al. (2024). "LABOR: Large Language Models for Orchestrating Bimanual Robots." _Humanoids 2024_.
13. Saccon, E. et al. (2025). "A Temporal Planning Framework for Multi-Agent Systems via LLM-Aided Knowledge Base Management." _RAL / ICRA 2025_.
14. Yang, Z. et al. (2025). "AutoHMA-LLM: Efficient Task Coordination and Execution in Heterogeneous Multi-Agent Systems." _ICRA 2025_.
15. Liu, K. et al. (2025). "COHERENT: Collaboration of Heterogeneous Multi-Robot System with Large Language Models." _ICRA 2025_.
16. Huang, Y. et al. (2025). "Compositional Coordination for Multi-Robot Teams with Large Language Models." _ICRA 2025_.
17. Wang, Y. et al. (2025). "DART-LLM: Dependency-Aware Multi-Robot Task Decomposition and Execution." _ICRA 2025_.
18. Zhu, W. et al. (2025). "DEXTER-LLM: Dynamic and Explainable Coordination of Multi-Robot Systems." _IROS 2025_.
19. Deng, H. et al. (2025). "GMATP-LLM: A General Multi-Agent Task Dynamic Planning Method." _Preprint_.
20. Gupta, A. et al. (2025). "Generalized Mission Planning for Heterogeneous Multi-Robot Teams via LLM-Constructed Hierarchical Trees." _ICRA 2025_.
21. Zhang, X. et al. (2025). "LaMMA-P: Generalizable Multi-Agent Long-Horizon Task Allocation and Planning." _ICRA 2025_.
22. Obata, K. et al. (2025). "LiP-LLM: Integrating Linear Programming and Dependency Graph with LLMs." _IEEE RA-L_.
23. Obi, I. et al. (2025). "SafePlan: Leveraging Formal Logic and CoT Reasoning for Enhanced Safety." _arXiv_.
24. Grigorev, D. et al. (2025). "VerifyLLM: LLM-Based Pre-Execution Task Plan Verification for Robots." _arXiv_.
25. Khan, A. et al. (2025). "Safety Aware Task Planning via Large Language Models in Robotics." _IROS 2025_.
26. Tian, X. et al. (2025). "LLM-CBT: LLM-Driven Closed-Loop Behavior Tree Planning for Heterogeneous UAV-UGV Swarms." _IROS 2025_.
27. Rabiei, M. et al. (2025). "LTLCodeGen: Code Generation of Syntactically Correct Temporal Logic." _IROS 2025_.
28. Xu, H. et al. (2025). "NL2HLTL2PLAN: Scaling Up Natural Language Understanding for Multi-Robots." _RA-L_.
29. Wang, S. et al. (2025). "Probabilistically Correct Language-Based Multi-Robot Planning Using Conformal Prediction." _RA-L_.
30. Yuan, Z. et al. (2025). "Trustworthy Robot Behavior Tree Generation Based on Multi-Source Heterogeneous Knowledge Graph." _ICRA 2025_.
31. Yu, J. et al. (2025). "CLGA: A Collaborative LLM Framework for Dynamic Goal Assignment in Multi-Robot Systems." _IROS 2025_.
32. Wan, X. et al. (2025). "Hierarchical Multi-Robot Task Planning with Fine-Tuned LLM." _IROS 2025_.
33. Wang, L. et al. (2025). "LLM-Based Cross-Regional Multi-Robot Task Planning with Scene Graphs and LTL." _IROS 2025_.
34. Chen, Z. et al. (2025). "Multi-Agent Systems for Robotic Autonomy with Reinforcement Learning." _IROS 2025_.
35. Yano, T. et al. (2025). "ICCO: MARL Coordinator for Language-Guided Multi-Robot Systems with CTDE." _Preprint_.
36. Song, Y. et al. (2025). "Flexible Constructivism Reflection Framework for Long-Horizon LLM Planning." _Preprint_.
37. Sun, H. et al. (2025). "Survey on LLM-Based Multi-Agent Decision Making." _Preprint_.
38. Reasoner, A. et al. (2025). "Theory of Mind and LLM Attention Selection for Implicit MRS Coordination." _Preprint_.
39. Zuzuárregui, A. & Carpin, S. (2025). "LLM Mission Planning for Precision Agriculture with IEEE Standard." _Preprint_.
40. Wang, T. et al. (2025). "AutoMisty: Multi-Agent LLM Framework for Code Generation on Social Robot." _Preprint_.
41. Kwon, S. et al. (2025). "LLM-Based Task Planning for Abstract Commands Using Hierarchical Scene Graphs." _Preprint_.
42. Hoffmeister, J. et al. (2025). "Zero-Knowledge Task Planning Using LLM-Generated Behavior Trees with Refinement Loop." _Preprint_.
43. Mu, Y. et al. (2025). "Look Before You Leap: Using Serialized State Machine for Language Conditioned Robotic Manipulation." _IROS 2025_.
44. Tang, Z. et al. (2025). "OBiMan-Bench & OBiMan-Planner: Open-World Task Planning for Humanoid Bimanual Dexterous Manipulation." _Preprint_.

