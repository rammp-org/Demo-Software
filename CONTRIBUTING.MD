# Contributing to Demo-Software

This document outlines the workflow for contributing code to the `Demo-Software` repo, for the April 2026 Demo. Having multiple people across multiple organizations developing code can lead to a lot of headache and can introduce unexpected bugs during integration. The goal of this workflow is to:

*   **Ensure code stability:** We want to make sure that we always have a stable, working version of the codebase ready to deploy. New contributions should never prevent us from testing existing functionality.
*   **Maintain Velocity:** As more people work on the code, we want to make sure we're not wasting our time dealing with merge conflicts.
*   **Facilitate Collaboration:** Clear standards for branching, naming, and committing help every team member understand the state of the codebase.
*   **Minimize integration bugs:** A clear flow for contributing code makes sure that all changes can be tested effectively before being merged into the larger project.

# Repo Organization
The repo will always have at least 2 branches:

1. `main`: This contains fully-tested, demo ready code. The code on this branch should be stable and deployable to the MeBot at all times. `main` is locked and is only updated periodically from `dev` (except for Hotfixes).

2. `dev`: This branch is the staging ground for testing new code and should be mostly stable. When contributing new code, you will open pull requests (PRs) to `dev` with your (tested) code.

3. Features/Bug branches: These are your active development branches and don't need to be stable. Every time you start developing a new feature, you will fork a feature/bug branch from the latest version of `dev`. You can do whatever you want in this branch! When your feature is ready, you open a PR back to `dev`.

# Contributing code

All of the features that need to be developed and bugs that need to be squashed should be added as issues on the Github repo. If the code you are working on is not listed as an issue, please open an issue before you begin this workflow.

### 1. Assign yourself to the issue on Github:

This ensures that two people are not developing the same feature simultaneously.

### 2. Pull the latest changes to your local repo:
In your terminal, navigate to the `Demo-Software` repo, make sure that your working tree is clean (with `git status`), and run:

```bash
git checkout dev
git pull
```

This will ensure that you are developing on the latest version of `dev`.

###  3. Create a feature branch:

Once you are on the latest `dev` branch, create a new branch for your feature or bug fix by running:

```bash
git checkout -b feature/[ISSUE NUMBER]-[BRIEF-DESCRIPTION]
```

If you are working on a bug, you can name your branch:

```bash
git checkout -b bug/[ISSUE NUMBER]-[BRIEF-DESCRIPTION]
```
The issue number is the big number right next to the title of the issue on Github. Keep branch names descriptive and concise.

### 4. Develop and Test:

Work on your changes in your feature branch. Feel free to commit code (and push to Github) as frequently as you'd like. Your feature branch is yours to work in, so use it however you'd like!  Ensure that you are testing your code locally on the hardware or using available simulations as applicable.


### 5. Open a pull request:

Once you have sufficiently tested your code, it's time to open a pull request into `dev` to integrate your feature!

First, you should rebase your latest changes onto `dev`. This applies your commits to the latest version of `dev` so that there are not merge conflicts if `dev` has changed since you started developing. You can do this by running:

First, you should merge the latest changes in `dev` into your feature branch. This ensures that you resolve merge conflicts before opening your pull request. You can do this by running:

```bash
git checkout dev
git pull origin dev
git checkout [YOUR FEATURE BRANCH]
git merge dev
```

After rebasing, you can then push your latest changes to Github by running:

```bash
git push origin [YOUR FEATURE BRANCH]
```

If the rebase applied changes, you may have to run:

```bash
git push origin [YOUR FEATURE BRANCH] --force-with-lease
```

Finally, go to repo on Github and create a new pull request from your feature branch to `dev`. Provide a brief description of the feature, your test procedures, and your test results in the description. If you include the phrase `Closes #[Issue number]`, the PR will automatically be linked to the issue.

Before merging a pull request, you will need to get somebody to review your code, so go ahead and assign a relevant reviewer.

### 6. Merging the Pull Request

Once you have gotten approval from the reviewers, you can merge your PR into `dev`! Please use `Squash and Merge` so that all of your intermediate commits don't get added to the development history on Github. After merging, remember to go back to your local `dev` branch and pull the changes to start your next feature!


# Hotfixes
**Use this only for critical bugs found in main that need an immediate fix.**

If a bug is found on `main` and `dev` is currently unstable or contains unfinished features, we use a Hotfix.

1. Branch off `main`:
```
Bash
git checkout main
git pull origin main
git checkout -b hotfix/critical-sensor-fix
```
2. Fix and Test: Ensure it fixes the immediate problem without breaking other main functionality.

3. Merge to main: Open a PR directly to main. Once merged, deploy to the MeBot.

**Back-port to dev: CRITICAL.** Immediately merge the hotfix branch (or main) into dev so the bug doesn't reappear in the next release cycle.

# FAQ

## Do I lose my history with Squash and Merge?
The git log on the server stays clean (one commit per feature), but the GitHub PR Archive preserves every individual commit and comment you made during development.