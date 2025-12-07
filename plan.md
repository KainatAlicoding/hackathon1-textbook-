# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-textbook-spec` | **Date**: 2025-12-05 | **Spec**: specs/001-textbook-spec/spec.md
**Input**: Feature specification from `/specs/001-textbook-spec/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the development of an AI-native textbook for physical AI and humanoid robotics, featuring an integrated RAG Chatbot, personalized content, and Urdu translation. The project will be built using Docusaurus for the frontend, Python FastAPI for the backend, Qdrant Cloud for vector database, Neon Postgres for relational database, and Better-Auth for user authentication. The development will proceed in five logical phases: Foundation & Setup, Content Architecture, RAG Chatbot Integration, Advanced Features, and Deployment.

## Technical Context

**Language/Version**: Python 3.9+ (for FastAPI, OpenAI Agents/ChatKit SDK), JavaScript/TypeScript (for Docusaurus, frontend components)
**Primary Dependencies**: Docusaurus, FastAPI, OpenAI Agents/ChatKit SDK, Qdrant Client, Psycopg2 (or similar for Neon Postgres), Better-Auth client libraries
**Storage**: Qdrant Cloud (vector embeddings for RAG), Neon Postgres (user data, content metadata, learning paths)
**Testing**: Jest/React Testing Library (frontend), Pytest (backend)
**Target Platform**: Web (deployed to GitHub Pages for Docusaurus frontend), Linux server (for FastAPI backend)
**Project Type**: Hybrid (static site + backend API)
**Performance Goals**: RAG Chatbot response time under 5 seconds (SC-003), Personalized content loading without noticeable delay (implicit in SC-001)
**Constraints**: Utilize Qdrant Cloud (Free Tier), Neon Postgres. Frontend deployment to GitHub Pages. Better-Auth for all authentication.
**Scale/Scope**: Target initial release for 13-week curriculum content, supporting personalization for individual users, real-time Urdu translation toggle, and interactive RAG chatbot.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **I. Hands-on Learning First**: Plan emphasizes practical examples and exercises as per content architecture phase.
- [x] **II. Personalized Learning Pathways**: Dedicated phase for implementing personalization logic.
- [x] **III. Multilingual Accessibility (Urdu Priority)**: Dedicated phase for Urdu translation toggle.
- [x] **IV. AI-Native & Chatbot Integration**: Dedicated phase for RAG Chatbot integration.
- [x] **V. Modern Tech Stack Adherence**: Plan explicitly uses Docusaurus, FastAPI, OpenAI SDK, Qdrant, Neon Postgres, Better-Auth.

## Project Structure

### Documentation (this feature)

```text
specs/001-textbook-spec/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── docs/                  # Docusaurus markdown content for modules/chapters
├── src/
│   ├── components/        # React components (e.g., PersonalizeButton, TranslateButton, ChatbotWidget)
│   ├── pages/
│   └── theme/             # Docusaurus theme overrides for custom layouts/buttons
└── docusaurus.config.js   # Docusaurus configuration for sidebar, plugins

backend/
├── app/                   # FastAPI application
│   ├── api/               # API endpoints (e.g., /personalize, /translate, /chat)
│   ├── core/              # Core logic, configurations
│   ├── db/                # Database connection, models, migrations
│   ├── schemas/           # Pydantic models for request/response validation
│   └── services/          # Business logic, external API integrations (Qdrant, OpenAI, Better-Auth)
├── tests/                 # Backend tests (unit, integration)
└── requirements.txt       # Python dependencies

```

**Structure Decision**: The project will adopt a monorepo-like structure with `frontend/` and `backend/` top-level directories. `frontend/` will house the Docusaurus application and its content, while `backend/` will contain the Python FastAPI services. This separation aligns with the distinct technology stacks and deployment targets while keeping related feature documentation together.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| N/A | N/A | N/A |
