# CyberScape backend container image.
# Multi-stage build: dependency installation stage + lean runtime stage.
FROM python:3.11-slim AS deps

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends build-essential libpq-dev libhiredis-dev curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY pyproject.toml ./
COPY src ./src
RUN pip install --upgrade pip \
    && pip install ".[dev]"

FROM python:3.11-slim AS runtime

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app/src

WORKDIR /app

RUN groupadd -r appuser \
    && useradd -r -g appuser appuser

COPY --from=deps /usr/local/lib/python3.11/site-packages /usr/local/lib/python3.11/site-packages
COPY --from=deps /usr/local/bin /usr/local/bin

COPY src ./src

USER appuser

# Entrypoint commands are defined per service in docker-compose.yml:
#   orchestrator: uvicorn cyberscape.main:app --host 0.0.0.0 --port 8000
#   agent_worker: python -m cyberscape.langgraph.worker