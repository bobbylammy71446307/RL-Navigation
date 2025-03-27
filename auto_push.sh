#!/bin/bash

# Configuration
TARGET_REPO="your-repo"       # Remote name for the fork (e.g., "upstream" or "anuragroy2001")
TARGET_BRANCH="Bob-branch"    # Branch name in the fork
MAIN_BRANCH="main"            # Your default branch
GIT_USERNAME="bobbylammy71446307"  # Your GitHub username
TOKEN_FILE="$HOME/Desktop/token.txt"  # Path to GitHub token

# Check if token file exists
if [ ! -f "$TOKEN_FILE" ]; then
    echo "❌ Error: GitHub token file not found at $TOKEN_FILE"
    exit 1
fi

# Read token securely
GITHUB_TOKEN=$(cat "$TOKEN_FILE" | tr -d '\n')

# Get commit message from user
read -p "Enter commit message: " COMMIT_MESSAGE
if [ -z "$COMMIT_MESSAGE" ]; then
    echo "❌ Error: Commit message cannot be empty!"
    exit 1
fi

# Check for changes
if [ -z "$(git status --porcelain)" ]; then
    echo "⚠️ No changes to commit."
    exit 0
fi

# Stage all changes
git add .

# Commit
git commit -m "$COMMIT_MESSAGE"

# Push to origin (your main repo)
echo "🚀 Pushing to origin/$MAIN_BRANCH..."
git push "https://$GITHUB_TOKEN@github.com/$GIT_USERNAME/ME5413_Final_Project.git" $MAIN_BRANCH

# Push to fork (target repo)
echo "🚀 Pushing to $TARGET_REPO/$TARGET_BRANCH..."
git push "https://$GITHUB_TOKEN@github.com/anuragroy2001/ME5413_Final_Project.git" $MAIN_BRANCH:$TARGET_BRANCH

echo "✅ Successfully pushed to both repositories!"
