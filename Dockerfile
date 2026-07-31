FROM registry.robotics-lab.ru/ros-demo:v2
ARG TARGETARCH

# OpenVSCode Server
ARG OPENVSCODE_DOWNLOAD_URL="https://github.com/gitpod-io/openvscode-server/releases/download/openvscode-server-v1.109.5/openvscode-server-v1.109.5-linux-${TARGETARCH/amd64/x64}.tar.gz"
ARG OPENVSCODE_ROOT="/opt/openvscode-server"
ENV PATH="${PATH}:${OPENVSCODE_ROOT}/bin"
RUN mkdir ${OPENVSCODE_ROOT} && curl -sSL "${OPENVSCODE_DOWNLOAD_URL}" | tar -xz -C ${OPENVSCODE_ROOT} --strip-components=1

# Default settings
RUN mkdir -p /root/.openvscode-server/data/Machine && echo '{\n\
    "workbench.colorTheme": "Default Dark Modern",\n\
    "files.dialog.defaultPath": "/src",\n\
    "python.languageServer": "Jedi",\n\
}' >> /root/.openvscode-server/data/Machine/settings.json

# Extensions
RUN mkdir -p /tmp/extensions && cd /tmp/extensions \
    && curl -sSL -O https://github.com/spkane/vscode-training-tweaks/releases/download/v0.0.3/training-tweaks-0.0.3.vsix \
    && openvscode-server --install-extension /tmp/extensions/* \
    && openvscode-server --install-extension ms-python.python \
    && openvscode-server --install-extension ms-python.black-formatter

# Entrypoint
ADD docker/entrypoint.sh /root/entrypoint.sh
WORKDIR /src
ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["/root/entrypoint.sh"]
