#!/usr/bin/env bash
# converge-release.sh <release-commit> [tag-message]
# Point BOTH the release tag and main at a release commit, so moonraker's
# branch-tracking update_manager AND git-describe's version string agree.
# Moving the tag ALONE leaves moonraker is_valid:False ("diverged / ahead N")
# because update_manager tracks primary_branch:main, not the tag.
set -euo pipefail
TAG="v0.13.0-core-one.1"      # the moving release tag (fork convention)
REMOTE="release"
COMMIT="${1:?usage: converge-release.sh <commit-ish>}"
COMMIT="$(git rev-parse --verify "${COMMIT}^{commit}")"

# --no-tags: we only need the remote's main to compute BASE; we force-push our OWN
# tag below. Fetching --tags would hit a "would clobber existing tag" REJECTION
# (non-zero) whenever the local tag is already moved ahead of the remote (the normal
# case here) and, under `set -e`, silently abort the whole converge before pushing.
git fetch "$REMOTE" --no-tags --quiet
BASE="$(git rev-parse "${REMOTE}/main")"
# main must fast-forward (linear) — refuse a non-FF so we never rewrite history
git merge-base --is-ancestor "$BASE" "$COMMIT" \
  || { echo "ABORT: $COMMIT is not a fast-forward of ${REMOTE}/main ($BASE)"; exit 1; }

# Annotated (-a), not lightweight: the fork's tag convention, and plain
# `git describe` (no --tags) only sees annotated tags. Optional 2nd arg = message.
MSG="${2:-release convergence -> $(git rev-parse --short "$COMMIT")}"
git tag -f -a "$TAG" -m "$MSG" "$COMMIT"
git push --force "$REMOTE" "$TAG"     # moving tag: force is expected
git push "$REMOTE" "$COMMIT:main"     # FF main: no --force; fails loudly if not FF
echo "converged: $TAG + ${REMOTE}/main -> $(git rev-parse --short "$COMMIT")"
