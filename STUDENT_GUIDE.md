# Student Guide for Physical AI & Humanoid Robotics Textbook

This guide contains manual action items you'll need to complete to fully deploy and configure the application.

## 1. API Keys Setup 🔑

### Getting Google Gemini API Key
1. Go to [Google AI Studio](https://aistudio.google.com/)
2. Sign in with your Google account
3. Click on "Get API Key" or navigate to the API keys section
4. Create a new API key or use an existing one
5. Copy the API key (it will look like `AIxxxxxxxxx`)

### Local Setup (.env file)
1. Create a `.env` file in the `backend/` directory if it doesn't exist
2. Add the following line to the file:
   ```
   GOOGLE_API_KEY=your_actual_api_key_here
   ```
   Replace `your_actual_api_key_here` with your actual Gemini API key.

### Vercel Deployment Setup
1. Go to your Vercel dashboard
2. Select your project
3. Go to Settings → Environment Variables
4. Add a new environment variable:
   - Key: `GOOGLE_API_KEY`
   - Value: Your actual Gemini API key
5. Save the changes
6. Redeploy your project for the changes to take effect

## 2. Moving to Cloud Database (Qdrant) ☁️

Currently, the code uses local storage for Qdrant. To move to Qdrant Cloud:

### Creating Qdrant Cloud Account
1. Go to [Qdrant Cloud](https://qdrant.tech/)
2. Sign up for a free account
3. Create a new cluster
4. Once created, you'll get a URL and API key in the dashboard

### Code Changes Required

**Files to modify:**
- `backend/app/ingest.py`
- `backend/api/chat.py`

**Before (Local Storage) - in both files:**
```python
# Initialize clients
qdrant_client = QdrantClient(
    path="./qdrant_data"  # Local storage
)
```

**After (Cloud Storage) - in both files:**
```python
# Initialize clients
qdrant_client = QdrantClient(
    url="your_qdrant_cloud_url",  # Replace with your Qdrant Cloud URL
    api_key=os.getenv("QDRANT_API_KEY")  # Add this to your .env file
)
```

**Additional Environment Variable:**
Add to your `.env` file (both locally and in Vercel):
```
QDRANT_API_KEY=your_qdrant_api_key
```

## 3. Vercel Deployment 🚀

### Environment Variables Required in Vercel
Add these environment variables in your Vercel project settings:

```
GOOGLE_API_KEY=your_gemini_api_key
QDRANT_API_KEY=your_qdrant_api_key  # Only if using Qdrant Cloud
```

### Deployment Steps
1. Connect your GitHub repository to Vercel
2. Make sure your project root is set correctly (should be the root of this repository)
3. Set the build command to: `cd frontend && npm install && npm run build`
4. Set the output directory to: `frontend/build`
5. Add the required environment variables as mentioned above
6. Deploy the project

### Important Notes
- Make sure to install dependencies in your frontend directory: `npm install`
- If using Qdrant Cloud, ensure your Qdrant cluster is active before deploying
- The backend API routes will be available under `/api/[endpoint]` in your deployed application