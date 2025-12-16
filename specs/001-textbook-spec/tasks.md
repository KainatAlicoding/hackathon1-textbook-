- [x] Initialize a new Docusaurus project (scaffold) in the current directory.
- [x] Clean up the default template (remove standard blog/docs folders).
- [x] Update `docusaurus.config.js` with the title "Physical AI & Humanoid Robotics".
- [x] Create the folder structure for the 4 Modules defined in the Specification.


<!-- phase 2 Content Creation** tasks -->
- [x] Configure `sidebars.js` to automatically generate the sidebar from the folder structure.
- [x] Create `frontend/docs/intro.md` with a "Course Overview" and "Prerequisites" based on the Constitution.
- [x] Write content for **Module 1 (ROS 2)**: Create markdown files for Weeks 1-5 inside `frontend/docs/module-01-ros2/` covering Nodes, Topics, and URDF.
- [x] Write content for **Module 2 (Digital Twin)**: Create markdown files for Weeks 6-7 inside `frontend/docs/module-02-digital-twin/` covering Gazebo physics and Unity integration.
- [x] Write content for **Module 3 (AI-Robot Brain)**: Create markdown files for Weeks 8-10 inside `frontend/docs/module-03-isaac-sim/` covering Isaac Sim and Nav2.
- [x] Write content for **Module 4 (VLA & Capstone)**: Create markdown files for Weeks 11-13 inside `frontend/docs/module-04-vla-capstone/` covering Voice-to-Action and the final project.

<!-- Phase 3: Backend Setup -->

- [x] Create a new `backend/` directory at the PROJECT ROOT level.
- [x] Create subdirectories: `backend/app/`, `backend/core/`, `backend/services/`.
- [x] Create `backend/requirements.txt` with required libraries (FastAPI, Qdrant, OpenAI, Postgres).
- [x] Create a basic `backend/app/main.py` to initialize FastAPI.
- [x] Create `backend/app/ingest.py` to read Markdown files from `frontend/docs` and chunk them for the Vector Database.

- [x] Refactor `backend/requirements.txt` and `backend/app/ingest.py` to replace OpenAI with **Google Gemini (`google-generativeai`)** for embeddings.
- [x] Create `backend/api/chat.py` to handle user questions using **Gemini Pro**. It must:
    1. Accept a user query.
    2. Create embedding using `models/embedding-001`.
    3. Search Qdrant for book context.
    4. Generate answer using `gemini-pro`.
- [x] Update `backend/app/main.py` to:
    1. Import and include the `chat` router from `backend.api.chat`.
    2. Add `CORSMiddleware` to allow requests from `http://localhost:3000` (Frontend).
    3. Ensure the root endpoint `/` returns a simple health check message like `{"status": "ok"}`.
- [x] Create a helper script `run_backend.py` (or instructions) at the root to easily start the uvicorn server.
- [x] Build a Floating Chatbot Widget for the Docusaurus project with: 1) src/components/Chatbot.js (React component), 2) src/components/Chatbot.css (Styles), 3) src/theme/Root.js (Integration) following all UI requirements (floating button, chat window, header, message list, input field, dark mode compatibility) and logic requirements (user input, thinking loader, POST request to backend, response display, sources chips).
- [x] Modernize the UI of the Docusaurus site with a cyberpunk robotics dark theme: 1) Update src/css/custom.css with modern dark theme using neon accents and glassmorphism effects, 2) Transform homepage hero section with new text and robot placeholder image, 3) Update HomepageFeatures with robotics-focused content, 4) Create first blog post about Physical AI textbook.
