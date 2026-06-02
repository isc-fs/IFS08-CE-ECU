# Repository Workflow

## Permanent Branches

This repository uses two long-lived branches:

- `main`: production-ready and validated code only.
- `dev`: integration branch for ongoing development.

Do not work directly on either of them.

## Working Branches

Create all new work from `dev` using one of these formats:

- `feat/<n>` for new functionality.
- `fix/<n>` for bug fixes.

The `feat` and `fix` counters are independent.

## Tracking Work With Issues

Each working branch should have one associated GitHub issue.

- Example branch: `feat/3`
- Example issue title: `[feat/3] Add CAN broadcast for mission state`

Open issues represent active work. Closed issues keep the historical record
after the branch is merged and deleted.

## Automation

The workflow in `.github/workflows/branch-issue.yml` manages issue tracking for
`feat/*` and `fix/*` branches:

- opens a tracking issue when the branch is first pushed
- applies the correct label
- warns if the branch number is not the expected next one
- fills the issue description from the first pushed commit message

## Typical Flow

1. Update `dev`.
2. Create `feat/<n>` or `fix/<n>` from `dev`.
3. Push the branch to GitHub.
4. Commit your work with a clear message.
5. Open a pull request targeting `dev`.
6. After review and merge into `dev`, delete the branch.
7. Merge `dev` into `main` only after full validation.
