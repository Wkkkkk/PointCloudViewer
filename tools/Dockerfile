FROM ubuntu:16.04 as builder
LABEL description="Builder of sample project on Ubuntu"

#environment variables
ENV PROJECT_DIR="/project" \
    BUILD_TYPE=RELEASE \
    CMAKE_VERSION=3.13.3 \
    PATH=/opt/cmake/bin:${PATH} \
    CMAKE_PREFIX_PATH=/usr/local/lib:/usr/local/lib64:${CMAKE_PREFIX_PATH} \
    LD_LIBRARY=/usr/local/lib:/usr/local/lib64:${LD_LIBRARY}

#basic dependencies.
#compile dependencies
#software-properties-common for add-apt-repository
#ca-certificates for verification
RUN apt-get update && apt-get install -y \
    software-properties-common \
    ca-certificates \
    build-essential \
    mesa-utils \
    glmark2 \
    cmake \
    sudo \
    vim \
    git \
    tar \
    unzip \
    wget \
    curl

#gcc8
RUN add-apt-repository -y ppa:jonathonf/gcc && \
    apt-get update && \
    apt-get install -y gcc-8 g++-8 && \
    rm -rf /usr/bin/gcc /usr/bin/g++ && \
    ln -s /usr/bin/g++-8 /usr/bin/g++ && \
    ln -s /usr/bin/gcc-8 /usr/bin/gcc

#cmake
RUN mkdir -p /opt &&\
    curl -jksSL https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.tar.gz | tar -xzf - -C /opt &&\
    mv -f /opt/cmake-${CMAKE_VERSION}-Linux-x86_64 /opt/cmake

#boost1.68
WORKDIR /3rdparty
RUN wget -O boost_1_68_0.tar.gz https://dl.bintray.com/boostorg/release/1.68.0/source/boost_1_68_0.tar.gz &&\
    tar xzvf boost_1_68_0.tar.gz &&\
    cd boost_1_68_0/ &&\
    ./bootstrap.sh --prefix=/usr/ &&\
    ./b2 &&\
    ./b2 install

#protobuf qt ssh eigen yaml-cpp
RUN apt-get update && apt-get install -y \
    libprotobuf-dev \
    protobuf-compiler \
    qt5-default \
    qttools5-dev-tools \
    libqt5opengl5-dev \
    libssh-dev \
    libeigen3-dev \
    libyaml-cpp-dev

WORKDIR /3rdparty
#osg(dynamic)
RUN git clone https://github.com/openscenegraph/OpenSceneGraph.git && \
    cd OpenSceneGraph && mkdir build && cd build \
    && cmake .. && make -j6 && make install

#draco(static)
RUN git clone https://github.com/google/draco.git && \
    cd draco && mkdir build && cd build \
    && cmake .. && make -j6 && make install

#clean up
RUN rm -rf /var/lib/apt/lists/* &&\
    chmod u+x /app/start.sh &&\
    mkdir /project

ADD ["start.sh", "/app/"]
VOLUME ["/project"]

WORKDIR /project/build
ENTRYPOINT ["/app/start.sh"]