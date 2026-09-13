# Dependency Locks

`pyproject.toml` is the source of dependency intent.

The committed lock files provide exact, hash-verified environments:

```text
requirements/runtime.lock
requirements/dev.lock
```

The runtime lock contains only deployed Python dependencies.

The development lock contains the runtime dependencies plus the tools used for
formatting, linting, type checking, testing, dependency auditing, lock
generation, pre-commit hooks, and package builds.

## Regenerate

From the repository root with the project development environment active:

```bash
./scripts/compile-locks.sh
```

The lock-generation script:

- currently requires Python 3.13;
- ignores host-level pip configuration while resolving;
- resolves against PyPI;
- generates SHA-256 hashes;
- avoids embedding host-specific package indexes such as PiWheels;
- records the canonical regeneration command in each generated lock.

Do not hand-edit lock files.

After changing dependency intent in `pyproject.toml`, regenerate both locks and
run the full repository verification before committing:

```bash
./scripts/compile-locks.sh
./scripts/verify.sh
```
