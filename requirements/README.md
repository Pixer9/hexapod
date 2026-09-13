# Dependency Locks

`pyproject.toml` is the source of dependency intent.

The committed lock files provide exact reproducible environments:

```text
requirements/runtime.lock
requirements/dev.lock
```

## Regenerate

From the repository root with the project development environment active:

```bash
pip-compile \
  --generate-hashes \
  --output-file=requirements/runtime.lock \
  pyproject.toml

pip-compile \
  --extra=dev \
  --generate-hashes \
  --output-file=requirements/dev.lock \
  pyproject.toml
```

Then verify that a fresh environment can install from each lock before
committing the regenerated files.

Do not hand-edit lock files.
