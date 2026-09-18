# Open-source release audit

Audit date: 2026-08-21

## Decision

The audited release is suitable for publication with its existing Git history,
branches, and tags. Project-owned code, configuration, static resources, and
documentation use Apache-2.0. The vendored Forge Tool protocol and ToolMessage
carrier retain their Apache-2.0 provenance as documented in
`THIRD_PARTY_NOTICES.md`; all other runtime dependencies resolve from public
package indexes.

## Completed checks

- Public dependency resolution succeeds with `uv sync --frozen --all-groups`.
- The former local path overrides for `forge-msgs` and `forge-kinematics` were
  replaced with the public `forge-msgs==1.0.1` and `forge-kinematics==1.0.1`
  from PyPI.
- The unpublished `forge-tool==0.1.0` protocol source and its required
  ToolMessage Arrow carrier are vendored under `src/forge_tool`, replacing the
  former local path dependency.
- Python is pinned to 3.12 via `.python-version`, matching the CI and release
  build environments.
- All 32 tests pass on Python 3.12 without physical hardware.
- `ruff check` passes on the publishable tree (vendored code excluded).
- The `forge-relative-motion-policy` CLI reports help correctly without extra
  environment setup.
- `pip-audit` reports no known vulnerabilities in locked runtime dependencies.
- `detect-secrets` reports zero findings in the git-tracked source tree.
- A `detect-secrets` scan across all 43 historical Git blobs reports zero
  findings.
- Private repository URLs and machine-specific paths are absent from the
  current publishable tree (a machine-specific absolute path in README.md
  was removed).
  Historical commits retain development provenance.
- No file approaches GitHub's 100 MiB hard limit; the largest tracked file
  (`uv.lock`) is under 160 KiB.
- The PyInstaller standalone build with the vendored protocol is verified in
  the Ubuntu 20.04 release workflow.
- GitHub Actions, Dependabot, issue, pull-request, contribution, and security
  policy files are included.

## License findings

- Project-owned Python, configuration, static resources, tests, and
  documentation: Apache-2.0.
- Vendored `forge-tool` and ToolMessage carrier snapshot: Apache-2.0.
- Runtime dependencies are not otherwise vendored and retain the licenses
  declared by their distributions:
  - `dora-rs`: MIT
  - `forge-msgs`: Apache-2.0
  - `forge-kinematics`: Apache-2.0
  - `numpy`: BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0
  - `pyarrow`: Apache-2.0
  - `pinocchio` (via `forge-kinematics`): MIT
  - `PyYAML`: MIT
- PyInstaller is a build-only dependency under GPL-2.0-or-later with its
  special exception.

See `THIRD_PARTY_NOTICES.md` for provenance and binary-distribution cautions.

## Known limitations

- This node is a Query provider: it resolves poses without moving the robot.
  Action execution safety remains the responsibility of downstream motion
  nodes and controllers.
- CI and this audit do not command physical hardware.
- The vendored protocol snapshot should be replaced with a public package after
  an equivalent compatible `forge-tool` release becomes available.
- PyInstaller binary releases bundle Pinocchio, NumPy, and PyArrow native
  libraries and require a separate artifact-level license and security review.
- The private vulnerability reporting URL in `.github/ISSUE_TEMPLATE/config.yml`
  targets the public `Forgelab-Robotics/operator-relative-pose-policy`
  repository.
- This audit is an engineering review, not legal advice, penetration testing,
  or safety certification.

## Publication model

The audited current tree is published together with the repository's existing
commit graph, branches, and tags. Historical commits are retained for
traceability and may contain obsolete internal repository locations; they must
not be treated as current installation instructions.
