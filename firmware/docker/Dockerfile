FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf \
    ca-certificates wget && rm -rf /var/lib/apt/lists/*

# Cross-compile FFTW3 float with NEON
RUN wget -qO- http://fftw.org/fftw-3.3.10.tar.gz | tar xz && \
    cd fftw-3.3.10 && \
    ./configure --host=arm-linux-gnueabihf --prefix=/usr/arm-linux-gnueabihf \
        --enable-float --enable-neon --disable-fortran \
        --enable-static --disable-shared && \
    make -j$(nproc) && make install && cd .. && rm -rf fftw-3.3.10

WORKDIR /src
