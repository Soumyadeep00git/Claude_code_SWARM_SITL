# ============================================================
# Stage 1: Build ArduCopter SITL binary from source
# ============================================================
FROM ubuntu:24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# ArduPilot build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3 python3-pip python3-dev \
    build-essential g++ gawk \
    libxml2-dev libxslt1-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone ArduPilot (specific stable tag for reproducibility)
ARG ARDUPILOT_TAG=Copter-4.5.7
RUN git clone --depth 1 --branch ${ARDUPILOT_TAG} \
    --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot.git /ardupilot

WORKDIR /ardupilot

# Install Python build deps (WAF needs these)
RUN pip3 install --break-system-packages future empy==3.3.4 pexpect

# Build ArduCopter for SITL
RUN ./waf configure --board sitl && ./waf copter

# ============================================================
# Stage 2: Slim runtime image
# ============================================================
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip \
    libxml2 libxslt1.1 \
    && rm -rf /var/lib/apt/lists/*

# Copy the built ArduCopter binary and default params
COPY --from=builder /ardupilot/build/sitl/bin/arducopter /ardupilot/build/sitl/bin/arducopter
COPY --from=builder /ardupilot/Tools/autotest/default_params/copter.parm \
                    /ardupilot/Tools/autotest/default_params/copter.parm

# Point config.py at the Docker ArduPilot location
ENV ARDUPILOT_DIR=/ardupilot

# Install Python dependencies
WORKDIR /app
COPY requirements.txt .
RUN pip3 install --break-system-packages --no-cache-dir -r requirements.txt

# Copy project code
COPY . .

# Ensure output and logs dirs exist
RUN mkdir -p /app/output /app/logs /app/models

# Default: run the GCS (override per-service in docker-compose)
CMD ["python3", "-m", "gcs.web_gcs"]
