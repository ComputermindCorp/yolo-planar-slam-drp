FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

# Install bootstrap tools
RUN apt-get update && \
    : "Common dependencies" && \
    apt-get install --no-install-recommends -y \
        build-essential \
        ca-certificates \
        cmake \
        git \
        sudo \
        unzip \
        vim \
        wget && \
    : "PCL dependencies" && \
    apt-get install --no-install-recommends -y \
        libboost-date-time-dev \
        libboost-filesystem-dev \
        libboost-iostreams-dev \
        libboost-system-dev \
        libboost-thread-dev \
        libeigen3-dev \
        libflann-dev \
        libvtk6-dev && \
    : "Pangolin dependencies" && \
    apt-get install --no-install-recommends -y \
        libgl1-mesa-dev \
        libglew-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff5-dev \
        libopenexr-dev \
        libgtk2.0-dev && \
    : "Socket viewer dependencies" && \
    apt-get install --no-install-recommends -y \
        autogen \
        autoconf \
        libtool \
        && curl -sL https://deb.nodesource.com/setup_12.x | bash - \
        && apt-get install -y --no-install-recommends \
        nodejs \
        libprotobuf-dev \
        protobuf-compiler && \
    rm -rf /var/lib/apt/lists/*

ARG USER_NAME
ARG USER_ID
ARG GROUP_ID

RUN groupadd -g ${USER_ID} ${USER_NAME}
RUN useradd --uid ${USER_ID} --gid ${GROUP_ID} -d /home/${USER_NAME} ${USER_NAME} -m -s /bin/bash
RUN gpasswd -a ${USER_NAME} sudo && gpasswd -a ${USER_NAME} dialout && gpasswd -a ${USER_NAME} video

# Set password
# RUN echo "${USER_NAME}:passw0rd" | chpasswd

# Do not set a password
RUN sed -i -e 's/%sudo\tALL=(ALL:ALL) ALL/%sudo   ALL=(ALL:ALL) NOPASSWD:ALL/' /etc/sudoers

# Set wget certificate
RUN echo 'ca-certificate = /etc/ssl/certs/ca-certificates.crt' >> /etc/wgetrc

ENV HOME=/home/${USER_NAME}
USER ${USER_NAME}

ARG NUM_BUILD_THREADS

# Build OpenCV
ENV OPENCV_VERSION=4.1.0

WORKDIR $HOME
COPY --chown=rns script/install/install_opencv.sh $HOME/script/install/install_opencv.sh
RUN wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip
RUN NUM_BUILD_THREADS=$NUM_BUILD_THREADS $HOME/script/install/install_opencv.sh
ENV OpenCV_DIR=$HOME/lib/${OPENCV_VERSION}/share/OpenCV

# Build nccn
ENV NCNN_VERSION=20220420

COPY --chown=rns script/install/install_ncnn.sh $HOME/script/install/install_ncnn.sh
RUN git clone https://github.com/Tencent/ncnn.git && \
    NUM_BUILD_THREADS=$NUM_BUILD_THREADS $HOME/script/install/install_ncnn.sh

# Build PCL
ENV PCL_VERSION=1.9.1

COPY --chown=rns script/install/install_pcl.sh $HOME/script/install/install_pcl.sh
RUN wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${PCL_VERSION}.tar.gz && \
    NUM_BUILD_THREADS=$NUM_BUILD_THREADS $HOME/script/install/install_pcl.sh

# Build Pangolin
ENV PANGOLIN_VERSION=dd801d244db3a8e27b7fe8020cd751404aa818fd

COPY --chown=rns script/install/install_pangolin.sh $HOME/script/install/install_pangolin.sh
WORKDIR $HOME
RUN git clone https://github.com/stevenlovegrove/Pangolin.git \
    && NUM_BUILD_THREADS=$NUM_BUILD_THREADS $HOME/script/install/install_pangolin.sh

# build socket.io-client-cpp
ENV SIO_CLIENT_VERSION=ff6ef08e45c594e33aa6bc19ebdd07954914efe0

COPY --chown=rns script/install/build_socket.io-client-cpp.sh $HOME/script/install/build_socket.io-client-cpp.sh
RUN git clone https://github.com/shinsumicco/socket.io-client-cpp.git && \
    NUM_BUILD_THREADS=$NUM_BUILD_THREADS $HOME/script/install/build_socket.io-client-cpp.sh
USER root
RUN cd $HOME/socket.io-client-cpp/build && \
    make install && \
    rm -rf * && \
    ldconfig
USER ${USER_NAME}

# build AIPoC2N
WORKDIR $HOME

RUN mkdir $HOME/yolo-planar-slam
COPY --chown=rns AIPoC2N $HOME/yolo-planar-slam/AIPoC2N
RUN cd $HOME/yolo-planar-slam/AIPoC2N/lib \
    && make clean \
    && make \
    && cd $HOME/yolo-planar-slam/AIPoC2N \
    && make -f Makefile.board clean \
    && make -f Makefile.board

# Build YOLO-Planar-SLAM
WORKDIR $HOME
COPY --chown=rns Examples $HOME/yolo-planar-slam/Examples
COPY --chown=rns Thirdparty $HOME/yolo-planar-slam/Thirdparty
COPY --chown=rns Vocabulary $HOME/yolo-planar-slam/Vocabulary
COPY --chown=rns include $HOME/yolo-planar-slam/include
COPY --chown=rns src $HOME/yolo-planar-slam/src
COPY --chown=rns drp_modules $HOME/yolo-planar-slam/drp_modules
COPY --chown=rns drp_ai_modules $HOME/yolo-planar-slam/drp_ai_modules
COPY --chown=rns socket_modules $HOME/yolo-planar-slam/socket_modules
COPY --chown=rns CAPEAddition $HOME/yolo-planar-slam/CAPEAddition
COPY --chown=rns g2oAddition $HOME/yolo-planar-slam/g2oAddition
COPY --chown=rns CMakeLists.txt LICENSE.txt License-gpl.txt README.md build.sh build_ros.sh $HOME/yolo-planar-slam/

# Remove cache
RUN rm -rf $HOME/yolo-planar-slam/build && \
    rm -rf $HOME/yolo-planar-slam/Thirdparty/DBoW2/build && \
    rm -rf $HOME/yolo-planar-slam/Thirdparty/g2o/build

# Import settings
COPY --chown=rns script/setup.sh $HOME/script/setup.sh
RUN echo "source $HOME/script/setup.sh" >> ~/.bashrc \
    && echo "export NUM_BUILD_THREADS=$NUM_BUILD_THREADS" >> ~/.bashrc \
    && echo "echo NUM_BUILD_THREADS=\$NUM_BUILD_THREADS" >> ~/.bashrc

WORKDIR $HOME/yolo-planar-slam
RUN /bin/bash -c "source ~/.bashrc; \
    NUM_BUILD_THREADS=$NUM_BUILD_THREADS ./build.sh"

ARG YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION
LABEL version="${YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION}"
