# syntax=docker/dockerfile:1.7-labs
FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

ARG PKG_REPO="TUM-Aries-Lab/imu-module"
ARG VER="main"
ENV PKG_REPO=${PKG_REPO} \
    VER=${VER}

# Install system deps: git for UV to fetch GitHub repos
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
 && rm -rf /var/lib/apt/lists/*

# Install UV
RUN --mount=type=cache,target=/root/.cache \
    pip install uv

# Install GitHub package via UV
RUN --mount=type=cache,target=/root/.cache/uv \
    if [ "$VER" = "main" ]; then \
        uv pip install --system "git+https://github.com/${PKG_REPO}.git"; \
    else \
        uv pip install --system "git+https://github.com/${PKG_REPO}.git@${VER}"; \
    fi

WORKDIR /app

RUN printf '%s\n' \
"import importlib, os, sys" \
"pkg = os.environ['PKG_REPO'].split('/')[-1].replace('-', '_')" \
"m = importlib.import_module(pkg)" \
"print('✅ import ok:', getattr(m, '__version__', 'unknown'), 'on', sys.version)" \
> smoke.py

CMD ["python", "smoke.py"]
