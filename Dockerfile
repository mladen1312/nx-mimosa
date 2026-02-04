# ═══════════════════════════════════════════════════════════════════════════════
# NX-MIMOSA DOCKER IMAGE
# Production Multi-Domain Radar Tracking System
# ═══════════════════════════════════════════════════════════════════════════════
#
# Build: docker build -t nx-mimosa:latest .
# Run:   docker run -it nx-mimosa:latest python -c "from python.nx_mimosa_v4_unified import *; print('OK')"
#
# Multi-stage build for minimal production image
# ═══════════════════════════════════════════════════════════════════════════════

# ─────────────────────────────────────────────────────────────────────────────
# STAGE 1: Builder
# ─────────────────────────────────────────────────────────────────────────────
FROM python:3.11-slim-bookworm AS builder

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    g++ \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    && rm -rf /var/lib/apt/lists/*

# Create virtual environment
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python dependencies
COPY requirements.txt .
RUN pip install --upgrade pip && \
    pip install -r requirements.txt

# ─────────────────────────────────────────────────────────────────────────────
# STAGE 2: Production
# ─────────────────────────────────────────────────────────────────────────────
FROM python:3.11-slim-bookworm AS production

# Labels
LABEL org.opencontainers.image.title="NX-MIMOSA" \
      org.opencontainers.image.description="Production Multi-Domain Radar Tracking System" \
      org.opencontainers.image.version="1.0.0" \
      org.opencontainers.image.vendor="Nexellum d.o.o." \
      org.opencontainers.image.authors="Dr. Mladen Mešter <mladen@nexellum.com>" \
      org.opencontainers.image.licenses="AGPL-3.0" \
      org.opencontainers.image.source="https://github.com/mladen1312/nx-mimosa"

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PATH="/opt/venv/bin:$PATH" \
    NX_MIMOSA_HOME="/app"

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopenblas0 \
    liblapack3 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Copy virtual environment from builder
COPY --from=builder /opt/venv /opt/venv

# Create non-root user
RUN groupadd --gid 1000 nxmimosa && \
    useradd --uid 1000 --gid 1000 --shell /bin/bash --create-home nxmimosa

# Set working directory
WORKDIR /app

# Copy application code
COPY --chown=nxmimosa:nxmimosa python/ ./python/
COPY --chown=nxmimosa:nxmimosa rtl/ ./rtl/
COPY --chown=nxmimosa:nxmimosa docs/ ./docs/
COPY --chown=nxmimosa:nxmimosa tests/ ./tests/
COPY --chown=nxmimosa:nxmimosa setup.py pyproject.toml requirements.txt README.md ./

# Install package
RUN pip install -e .

# Switch to non-root user
USER nxmimosa

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import numpy; from python.nx_mimosa_v4_unified import NXMIMOSAUnified; print('OK')" || exit 1

# Default command
CMD ["python", "-c", "from python.nx_mimosa_v4_unified import NXMIMOSAUnified; print('NX-MIMOSA v1.0.0 ready')"]


# ─────────────────────────────────────────────────────────────────────────────
# STAGE 3: Development (Optional)
# ─────────────────────────────────────────────────────────────────────────────
FROM production AS development

USER root

# Install development dependencies
RUN pip install pytest pytest-cov black flake8 mypy

# Install Verilator for RTL simulation
RUN apt-get update && apt-get install -y --no-install-recommends \
    verilator \
    && rm -rf /var/lib/apt/lists/*

USER nxmimosa

# Override command for development
CMD ["bash"]


# ─────────────────────────────────────────────────────────────────────────────
# STAGE 4: CI/CD Runner
# ─────────────────────────────────────────────────────────────────────────────
FROM development AS ci

USER root

# Install CI tools
RUN pip install coverage codecov

# Run tests by default
CMD ["pytest", "tests/", "-v", "--cov=python", "--cov-report=xml"]
