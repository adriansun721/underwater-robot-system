# Domain Docs

How engineering skills should consume this repository's domain documentation.

## Before exploring, read these

- `CONTEXT.md` at the repository root.
- `CONTEXT-MAP.md`, if it exists, and each context relevant to the task.
- Relevant ADRs under `docs/adr/`.

If these files do not exist, proceed silently. The `/domain-modeling` skill
creates them when domain terms or architectural decisions are resolved.

## File structure

This repository uses a single-context layout:

```
/
|-- CONTEXT.md
|-- docs/adr/
`-- src/
```

## Use the glossary's vocabulary

When naming a domain concept in issues, proposals, hypotheses, or tests, use
the term defined in `CONTEXT.md`. Do not drift to synonyms that the glossary
explicitly avoids.

If a required concept is absent, reconsider whether the term belongs to the
project or record the gap for `/domain-modeling`.

## Flag ADR conflicts

If proposed work contradicts an existing ADR, surface the conflict explicitly
rather than silently overriding the decision.
