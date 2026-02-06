# NX-MIMOSA Production Container
# Multi-stage build for minimal image size
#
# Build:  docker build -t nx-mimosa:5.2.0 .
# Run:    docker run --rm nx-mimosa:5.2.0 python -c "from nx_mimosa import __version__; print(__version__)"
# Test:   docker run --rm nx-mimosa:5.2.0 pytest tests/ -v

# ---- Stage 1: Build ----
FROM python:3.12-slim AS builder

WORKDIR /build
COPY pyproject.toml README.md CHANGELOG.md LICENSE ./
COPY python/ python/
COPY nx_mimosa/ nx_mimosa/
COPY tests/ tests/

RUN pip install --no-cache-dir build && \
    python -m build --wheel && \
    pip install --no-cache-dir dist/nx_mimosa-*.whl

# ---- Stage 2: Production ----
FROM python:3.12-slim AS production

LABEL maintainer="Dr. Mladen Mešter <mladen@nexellum.com>"
LABEL org.opencontainers.image.title="NX-MIMOSA"
LABEL org.opencontainers.image.description="Adaptive Multi-Sensor Multi-Target Radar Tracker"
LABEL org.opencontainers.image.version="5.2.0"
LABEL org.opencontainers.image.vendor="Nexellum d.o.o."
LABEL org.opencontainers.image.licenses="AGPL-3.0-or-later"

# Copy installed packages from builder
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /build/tests /app/tests

WORKDIR /app

# Install pytest for container-based testing
RUN pip install --no-cache-dir pytest

# Non-root user for security
RUN useradd -m -r tracker && chown -R tracker:tracker /app
USER tracker

# Health check
HEALTHCHECK --interval=30s --timeout=5s CMD python -c "from nx_mimosa import MultiTargetTracker; print('OK')"

# Default: run tests
CMD ["pytest", "tests/", "-v", "--tb=short"]
