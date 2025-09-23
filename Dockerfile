FROM python:3.11-slim

# Install system dependencies for Raysect compilation
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    build-essential \
    pkg-config \
    python3-dev \
    cython3 \
    libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy project files
COPY . .

# Upgrade pip and install build tools first
RUN pip install --upgrade pip setuptools wheel

# Install Python dependencies
RUN pip install --no-cache-dir numpy matplotlib cython

# Install Raysect
RUN pip install --no-cache-dir raysect

# Default command
CMD ["python", "main.py"]