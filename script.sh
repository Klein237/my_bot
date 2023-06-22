#!/bin/bash



if ! command -v docker &> /dev/null
then
    echo "Docker n'est pas installé"
    echo "Installation de Docker..."

    # Mise à jour de la liste des packages disponibles
    sudo apt-get update

    # Installation de Docker
    sudo apt-get install docker.io -y

    # Ajout de l'utilisateur courant au groupe docker pour éviter les erreurs d'autorisation
    sudo usermod -aG docker $USER

    echo "Docker a été installé avec succès."
else
    echo "Docker est déjà installé."
fi

ABSOLUTE_DOCKER=`pwd`
CONTAINER_NAME=botcon
WS_PATH="${ABSOLUTE_DOCKER%%/src/*}"

mkdir ~/.ccache || echo ccache folder already exists

NAME=foxbot # replace by the name of your image


echo Stopping previous $CONTAINER_NAME
docker stop $CONTAINER_NAME || echo Container was not running
docker rm $CONTAINER_NAME || echo Image was not created


docker build -f Dockerfile -t $NAME .

docker run -it \
    --rm \
    --net=host \
    --ipc=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=4" \
    -h "$CONTAINER_NAME" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=unix$DISPLAY \
    -e XDG_RUNTIME_DIR=/home/klein/xdg \
    --add-host "$CONTAINER_NAME:127.0.0.1" \
    --privileged \
    --name $CONTAINER_NAME \
    -v $WS_PATH:/home/klein/ros_ws \
    -v $HOME/.ccache:/home/klein/.ccache \
    "${NAME}"









