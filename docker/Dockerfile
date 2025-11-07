#syntax=docker/dockerfile:1.7
FROM ros:jazzy-perception-noble

LABEL org.opencontainers.image.title="AUWO ROS2 Jazzy Dev"
LABEL org.opencontainers.image.description="ROS 2 Jazzy container for Jetson with clean UID/GID handling"
LABEL org.opencontainers.image.licenses="Apache-2.0"

# Build-time args so host file perms match inside the container
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG WORKSPACE=/workspace

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    WORKSPACE=${WORKSPACE}

SHELL ["/bin/bash", "-lc"]

# Core tools, ROS dev deps, locale, and rosdep cache
RUN set -eux \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      locales sudo ca-certificates curl wget git \
      build-essential cmake \
      python3-pip python3-colcon-common-extensions \
      python3-vcstool python3-rosdep udev \
 && rm -rf /var/lib/apt/lists/* \
 && sed -i 's/^# *\(en_US.UTF-8 UTF-8\)/\1/' /etc/locale.gen \
 && locale-gen en_US.UTF-8 \
 && rosdep update

# Ensure device groups exist (before adding user)
RUN set -eux; \
    for g in video dialout plugdev; do getent group "$g" >/dev/null || groupadd "$g"; done

# Idempotent user/group creation mapped to host UID/GID
RUN set -eux; \
    if getent group "${GROUP_ID}" >/dev/null; then \
        gname="$(getent group ${GROUP_ID} | cut -d: -f1)"; \
        [ "$gname" = "robot" ] || groupmod -n robot "$gname"; \
    else \
        groupadd -g "${GROUP_ID}" robot; \
    fi; \
    if getent passwd "${USER_ID}" >/dev/null; then \
        uname="$(getent passwd ${USER_ID} | cut -d: -f1)"; \
        [ "$uname" = "robot" ] || usermod -l robot -d /home/robot -m "$uname"; \
        usermod -g "${GROUP_ID}" robot; \
    else \
        useradd -m -u "${USER_ID}" -g "${GROUP_ID}" -s /bin/bash robot; \
    fi; \
    usermod -aG video,dialout,plugdev robot; \
    echo "robot ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/robot; chmod 0440 /etc/sudoers.d/robot

# Prepare workspace
RUN mkdir -p "${WORKSPACE}/src" && chown -R robot:robot "${WORKSPACE}"

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER robot
WORKDIR ${WORKSPACE}

# Convenience: auto-source ROS and overlay for interactive shells
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc \
 && echo 'if [ -f "${WORKSPACE}/install/setup.bash" ]; then source "${WORKSPACE}/install/setup.bash"; fi' >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
