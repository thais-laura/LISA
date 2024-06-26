# Adicionando ROS noetic desktop image como base
FROM osrf/ros:noetic-desktop-full

# Evitando problemas com atividades interativas
    # Assim evita aquelas confirmações de instalação de pacotes que podem aparecer.
    # Basicamente a gente seta que nosso sistema atual é não interativo
# Descomente as linhas abaixo caso haja problemas relacionados
# ARG DEBIAN_FRONTEND=noninteractive
# ENV TZ=America/Sao_Paulo

# Setando usuário no container
ARG USERNAME=lisa
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Configurando o usuário para ser um non-root
    # Evita problemas com os volumes (Pasta que são compartilhadas)
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Setando sudo
    # dando permissão de root a usuários não root
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Instalando software e pacotes, qualquer novo pacote adicionem aqui
## Uttils
RUN apt-get update \
    && apt-get install -y git-all

## Instalando rviz
RUN apt-get update \
    && apt-get install -y rviz\
    && rm -rf /var/lib/apt/lists/*

## Instalando biblicotecas de python
RUN apt-get update \
    && apt-get install -y python3 python3-pip \
    && apt-get install -y python-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN pip install opencv-python

RUN apt-get update \
    && apt-get install ffmpeg
    
# Install rosserial for communication with microcontrollers
RUN apt-get update && apt-get install -y \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial


# Configurando o entrypoint script e bashrc
COPY config/entrypoint.sh /entrypoint.sh
COPY config/bashrc /home/${USERNAME}/.bashrc
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
CMD [ "bash" ]
