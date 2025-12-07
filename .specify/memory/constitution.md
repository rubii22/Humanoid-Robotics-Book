<!--
Sync Impact Report:
Version change: N/A → 1.0.0
Added sections: All principles and sections based on user input
Removed sections: None
Templates requiring updates: ⚠ pending (.specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md)
Follow-up TODOs: None
-->
# AI/Spec-Driven Book Creation Using Docusaurus, Spec-Kit Plus, and Claude Code Constitution

## Core Principles

### Quality and Reproducibility
Quality, accuracy, structure, and reproducibility must guide every chapter, section, and example in the book. All content must be reproducible and verifiable by students.

### Specification-Driven Development
The entire book must be produced using specification-driven workflows with Spec-Kit Plus and Claude Code. Each chapter must be generated through: /sp.specify, /sp.clarify, /sp.plan, /sp.tasks, /sp.implement

### Technical Standards
All examples must run on: Node 20+, Docusaurus v3+, GitHub Pages. Code samples must be minimal, correct, and tested.

### Writing Standards
Write for students learning AI-native software development with Flesch-Kincaid grade 8–10 readability. Maintain consistency in structure and tone, ensuring accuracy of all technical claims and definitions. All tutorials must work exactly as written. No hallucinated tools, commands, or frameworks.

### Content Integrity
No API keys or credentials included anywhere. No copyrighted images or text. All code and content must be original or properly attributed. No hallucinated tools, commands, or frameworks.

### Documentation Standards
Use fenced code blocks for all code. Use headings consistently: H1 for chapters, H2/H3 for sections. Provide step-by-step guides.

## Success Criteria
Book builds successfully in Docusaurus with no errors. GitHub Pages deployment works without manual fixes. All chapters follow the constitution and specification workflow. Students can reproduce every command, tutorial, and example. Clear navigation, functioning sidebar, and consistent formatting.

## Deployment Standards
Book format: Markdown, compatible with Docusaurus. Deployment target: GitHub Pages.

## Governance
Constitution supersedes all other practices. All development must follow the specification-driven workflow. All PRs/reviews must verify compliance with the standards. Students must be able to reproduce every example.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
