FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ── Pre-configure tzdata so it never prompts ─────────────────
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && echo "UTC" > /etc/timezone

# ── System dependencies ──────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip python3-dev \
    git gcc g++ make sudo lsb-release software-properties-common \
    libxml2-dev libxslt1-dev tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages ──────────────────────────────────────────
RUN pip3 install --no-cache-dir pymavlink matplotlib pillow pyyaml

# ── Create non-root user (ArduPilot install script refuses root) ─
RUN useradd -m -s /bin/bash ardupilot \
    && echo "ardupilot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo 'Defaults env_keep += "DEBIAN_FRONTEND TZ"' >> /etc/sudoers

# ── Build ArduCopter SITL ────────────────────────────────────
USER ardupilot
ENV USER=ardupilot
ENV HOME=/home/ardupilot
RUN git clone --depth 1 --recurse-submodules \
    https://github.com/ArduPilot/ardupilot.git /home/ardupilot/ardupilot

WORKDIR /home/ardupilot/ardupilot

# Install ArduPilot build prerequisites
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Source the profile to get the right PATH for waf
SHELL ["/bin/bash", "-c"]
RUN source ~/.profile && ./waf configure --board sitl && ./waf copter

# Verify binary exists
RUN test -f /home/ardupilot/ardupilot/build/sitl/bin/arducopter

# ── Switch back to root for app setup ─────────────────────────
USER root

# ── Copy project code ────────────────────────────────────────
WORKDIR /app
COPY docker_sim/ /app/docker_sim/
COPY guidance_lib/ /app/guidance_lib/
COPY failsafe_lib/ /app/failsafe_lib/
COPY swarm_lib/ /app/swarm_lib/
COPY hierarchy.yaml /app/hierarchy.yaml
COPY docker/params/ /app/params/
COPY docker/entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# ── Environment defaults ─────────────────────────────────────
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/app
ENV ARDUCOPTER_BIN=/home/ardupilot/ardupilot/build/sitl/bin/arducopter
ENV COPTER_DEFAULTS=/home/ardupilot/ardupilot/Tools/autotest/default_params/copter.parm
ENV MP_PARAMS=/app/params/mission_planner.parm

ENTRYPOINT ["/app/entrypoint.sh"]
