FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

RUN useradd -m ubuntu

RUN <<EOF
    apt-get update
    apt-get -y full-upgrade
    apt-get install -y --no-install-recommends \
    build-essential gdb ninja-build git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl cmake black ccache
    rm -rf /var/lib/apt/lists/*
EOF

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

RUN <<EOF
    apt-get update
    apt-get -y full-upgrade
    apt-get install -y --no-install-recommends \
    ca-certificates
    rm -rf /var/lib/apt/lists/*
EOF

USER ubuntu

WORKDIR /opt/esmini
RUN chown ubuntu:ubuntu /opt/esmini

ADD --chown=ubuntu:ubuntu https://github.com/esmini/esmini.git#65082e52cfb5ec5bbca16f4a6bb322fa928395bc .

RUN <<EOF
    cmake -B build/ -S . \
    -DENABLE_COLORED_DIAGNOSTICS=ON \
    -DENABLE_WARNINGS_AS_ERRORS=ON \
    -DDOWNLOAD_EXTERNALS=OFF \
    -DENABLE_CCACHE=ON \
    -DBUILD_EXAMPLES=OFF \
    -DUSE_IMPLOT=OFF \
    -DUSE_OSG=OFF -DUSE_OSI=OFF -DUSE_SUMO=OFF -DUSE_GTEST=OFF
    cmake --build build/ --config Release --target install -j
EOF

RUN rm -f config.yml

WORKDIR /app
COPY ./pyproject.toml .
COPY ./uv.lock .
RUN uv sync --locked
COPY . .

ENV PORT=50051

ENTRYPOINT [ "/bin/bash" ]
CMD [ "/app/entrypoint.sh" ]
