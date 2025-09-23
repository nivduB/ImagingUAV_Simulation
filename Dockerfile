FROM python:3.11-slim

# Install system dependencies including git and ssh
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .
CMD ["python", "main.py"]FROM python:3.11-slim
WORKDIR /app
COPY . .
CMD ["python", "main.py"]
