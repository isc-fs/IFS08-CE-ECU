# IFS08 - CE-ECU

Embedded firmware for the IFS08 CE-ECU.

## Getting started

Create a GitHub account if you don't have one yet.

Download and install GitHub Desktop (beginner) or Git CLI (advanced).

If this is your first time using GitHub Desktop, make sure to read the User Manual. If this is your first time using Git, start with a tutorial. There are many available online:

- Git Tutorial
- Atlassian Git Tutorial

Keep a copy of GitHub's Git Cheat Sheet handy as a reference.

Clone this repository to your machine:

- SSH: `git@github.com:isc-fs/IFS08-CE-ECU.git`
- HTTPS: `https://github.com/isc-fs/IFS08-CE-ECU.git`

## How we work with this repository

### Main branches

The repository has two permanent branches:

- `main` is the production branch. It contains only validated code that can be used in the car. Never work directly on it.
- `dev` is the development branch. It is the integration point where everyone's work comes together. Never work directly on it either; all changes arrive through a feature branch.

```text
main  ------------------o----------------------o-->  (validated releases only)
                        ^                      ^
dev   ------o---o---o---o---o---o---o---o---o--o-->  (continuous integration)
            ^   ^       ^   ^   ^       ^   ^
          feat/1 fix/1 feat/2 fix/2   feat/3 fix/3
```

### Feature branches

All work, whether a new feature or a bug fix, is done on a feature branch created from `dev`. When the work is ready, a Pull Request is opened toward `dev`, reviewed, merged, and the branch is deleted.

There are two branch types, each with its own independent numeric counter:

- `feat/<n>` -> new functionality (`feat/1`, `feat/2`, `feat/3` ...)
- `fix/<n>` -> bug fix (`fix/1`, `fix/2`, `fix/3` ...)

The `feat` and `fix` counters are independent: `feat/2` and `fix/2` can exist at the same time with no conflict.

### Tracking branch history

Feature branches are deleted after merging to keep the repository clean. The history of each branch can be preserved in GitHub Issues.

Every branch should have one associated issue. The issue carries a label (`feat` or `fix`) and its title includes the branch number, for example: `[feat/3] Add CAN broadcast for mission state`. When the branch is merged and deleted, the issue is closed, becoming a permanent record of the work done.

To see which branches are currently active, filter issues by label and status `open`. To browse the full history, filter by label and status `closed`. The number for the next branch of each type is the last closed issue of that type plus one.

Example: if the last closed issue with label `feat` is `[feat/4] ...`, the next feature branch will be `feat/5`.

### Automation

This repository currently documents the workflow above, but it does not yet include the GitHub Actions automation described for other projects. If we want automatic issue creation and branch tracking here as well, that workflow still needs to be added.

## Step-by-step workflow

### 1. Create the branch

```bash
# Make sure you are on an up-to-date dev
git checkout dev
git pull origin dev

# Create your branch using the next available number for its type
# (last closed issue of that type + 1)
git checkout -b feat/5    # or fix/3, depending on that type's counter
```

To find the right number, go to Issues, filter by label `feat` or `fix`, sort by newest, and read the last number.

### 2. Push the branch

```bash
git push origin feat/5
```

If branch-tracking automation is added in the future, the tracking issue can be created automatically at this step. For now, create the corresponding issue manually if your team wants to keep that traceability.

### 3. Work and commit

```bash
# Make your changes and commit with a clear, descriptive message
git add .
git commit -m "short description of what this commit does"

# Push the changes
git push origin feat/5
```

Use clear first-commit messages so the branch purpose is easy to identify from both Git history and the related issue.

### 4. Open a Pull Request

When the work is ready, open a Pull Request on GitHub from your branch toward `dev`. In the PR description, write `Closes #<issue-number>` so the issue closes automatically when the PR is merged.

Before requesting a review, check that:

- The code compiles with no errors or warnings.
- You have tested the change on the bench if applicable.
- The PR targets `dev`, not `main`.

### 5. Review and merge

Another team member reviews the PR. Once approved, it is merged into `dev` and the branch is deleted. The issue is then closed as a permanent record.

### 6. Merging into `main`

When `dev` holds a set of validated changes that are ready for the car, a responsible team member opens a Pull Request from `dev` into `main`. This should only happen after full firmware validation.

ISC Racing Team - IFS08 CE-ECU
