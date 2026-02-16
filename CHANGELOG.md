# Changelog

## 2026-02-16
- Added related pdfs @[proposal/literature/added]
- Added changelog

## 2026-02-12

### Prompt
@literature
Make a Python script that extracts all these papers and converts them to txt.
Also make refactoring of the repo.

### Response
Created @txt.

### Prompt
Analyze the literature and make a summary in md file.

### Response
Created @literature_summary.md.

### Prompt
Now, we would like to develop a new architecture of heterogeneous multi-robot system as shown in the picture.
We assume that we have a multi-robot system in a partially observed environment.
The system can receive commands in natural languages from a human. It goes through an agentic reasoning layer which is connected to the multi-robot systems, also connected to external resources through MCP tools and agents, and is able to convert human intent into complex workflows to be executed by the multi-robot system. Any complex task can be executed by one robot or multiple robots.
The robots may or may not have access to external information. They are able to communicate with each other.
One of the most important contributions is that we will have a safety layer that will ensure the safe operation of any complex multi-task execution by the multi-robot system.
We will have also a real-time monitoring layer that will connect real available data during execution from the multi-robot system and external world.

We want to use formal logic verification on top of LLM reasoning to combine a rule-based approach (formal verification) and probabilistic LLM reasoning in our system.

For modeling multi-robot task planning, we can use general-purpose semi-structured data (JSON, YAML) or dedicated planning frameworks (PDDL or Behavior Tree which are more recent - you can propose other trending planning techniques if available), but we may use an abstract planning layer for now that can support multiple technologies like PDDL, behavior tree, or others.

Make me an md file that writes this idea clearly and systematically and contrasts it with the literature you summarized above.
This is a picture that also illustrates our idea: @proposal-idea.png.

You can also make a mermaid diagram.

### Response
Created @architecture_proposal.md.

### Prompt
Now, make me an md file that compares clearly with this paper and my idea and propose how to make my contribution more substantial and not just incremental for IROS and ICRA.

### Response
Created @competitive_analysis.md.
