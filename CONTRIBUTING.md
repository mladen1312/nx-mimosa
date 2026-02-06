# Contributing to NX-MIMOSA

Thank you for your interest in contributing to NX-MIMOSA! This document provides guidelines for contributing to the project.

## Code of Conduct

By participating in this project, you agree to maintain a respectful and professional environment.

## How to Contribute

### Reporting Bugs

1. Check if the issue already exists in [GitHub Issues](https://github.com/mladen1312/nx-mimosa/issues)
2. Use the bug report template
3. Include: Python version, NumPy version, minimal reproducible example, expected vs actual behavior

### Feature Requests

Open an issue with the `enhancement` label. Include:
- Use case description
- Expected behavior
- Relevant domain (military, ATC, automotive, space, maritime)

### Code Contributions

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Write tests (pytest) for all new functionality
4. Ensure all tests pass: `pytest tests/ -v`
5. Submit a pull request

### Development Setup

```bash
git clone https://github.com/mladen1312/nx-mimosa.git
cd nx-mimosa
pip install -e ".[dev]"
pytest tests/ -v
```

### Code Standards

- **Python 3.9+** compatibility required
- **Type hints** on all public functions
- **Docstrings** with Args/Returns/Reference sections
- **Traceability** comments: `// [REQ-XXX]` for requirements tracking
- **NumPy only** — no additional runtime dependencies
- **Tests** for every new function (pytest, assert-based)

### Testing

```bash
# Full test suite
pytest tests/ -v

# Specific module
pytest tests/test_nx_mimosa_v50.py -v

# Intelligence + Fusion
pytest tests/test_intelligence_fusion.py -v
```

### Documentation

- API docs use Sphinx autodoc (NumPy docstring style)
- Build docs: `sphinx-build -b html docs/source docs/build/html`

## Licensing

### Open Source (AGPL v3)

Contributions to the open-source version are accepted under AGPL v3. By submitting a pull request, you agree that your contribution will be licensed under the same terms.

### Contributor License Agreement (CLA)

For contributions that may be included in commercial versions, we require a simple CLA. Contact mladen@nexellum.com for details.

### What Can Be Contributed

| Area | Open Source | Commercial Only |
|---|---|---|
| Bug fixes | ✅ | — |
| Motion models | ✅ | — |
| Metrics | ✅ | — |
| Coordinate transforms | ✅ | — |
| Association algorithms | ✅ | — |
| Platform database entries | — | ✅ |
| FPGA RTL | — | ✅ |
| DO-254 certification artifacts | — | ✅ |

## Contact

- **Technical**: mladen@nexellum.com
- **Commercial**: mladen@nexellum.com
- **Issues**: https://github.com/mladen1312/nx-mimosa/issues

## Recognition

All contributors are acknowledged in the CHANGELOG and release notes.
